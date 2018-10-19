local torch = require 'torch'
local pcl = require 'pcl'
local x3d = require 'x3d.init'
local cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'colormap'      -- luarocks install colormap
local cap = require 'ximea_nodes'

local HeightAnalysis = torch.class('ximea_nodes.HeightAnalysis', cap)


function HeightAnalysis.getDefaultParameters()
    return {
        -- first filter run on input data
        crop_box = {
            min = { -1.5, -1.5, 0.5 },      -- min point for crop box
            max = { 1.5, 1.5, 0.85 }        -- max point for crop box
        },
        random_sampling_portion = 0.05,
        plane_segmentation = {
            method = pcl.SAC.RANSAC,
            distance_threshold = 0.003,
            max_iterations = 100,
            probability = 0.99,
        },
        -- statistical outlier removal
        outlier_removal = {
            mean_k = 16,
            standard_deviation_threshold = 0.01,
        },
        height_map = {
            min_z = -0.005,
            max_z = 0.05
        }
    }
end


function HeightAnalysis:__init(parameters)
    self.parameters = parameters or HeightAnalysis.getDefaultParameters()
end



local function findDominantPlane(self, cloud)
    local seg = pcl.SACSegmentation(cloud.pointType)
    seg:setOptimizeCoefficients(true)
    seg:setModelType(pcl.SACMODEL.PLANE)
    seg:setMethodType(self.parameters.plane_segmentation.method)
    seg:setDistanceThreshold(self.parameters.plane_segmentation.distance_threshold)
    seg:setMaxIterations(self.parameters.plane_segmentation.max_iterations)
    seg:setOptimizeCoefficients(true)
    seg:setProbability(self.parameters.plane_segmentation.probability)
    seg:setInputCloud(cloud)

    local inliers = pcl.Indices()
    local coefficients = torch.FloatTensor()

    seg:segment(inliers, coefficients)

    return coefficients, inliers
end


local function randOrthogonal(axis)
    assert(axis:norm() > 1e-5, 'axis parametre must not be null-vector')

    -- sample random vectors and select with lowest scalar product
    local r = torch.randn(10, 3)
    local s = (r - axis:repeatTensor(1, 10)) * axis
    local _, i = s:min(1)
    local ortho = x3d.normalize(torch.cross(r[i[1]], axis))
    return ortho
end


function HeightAnalysis:generateHeightMap(cloud)

    local cropped = pcl.filter.cropBox(
        cloud,              -- input
        self.parameters.crop_box.min,   -- min
        self.parameters.crop_box.max,   -- max
        nil,                -- rotation
        nil,                -- translation
        torch.eye(4),       -- transform
        nil,                -- indices
        nil,                -- output
        false,              -- negative
        nil,                -- removed_indices
        true                -- keep_organized
    )

    -- remove nan indices
    local c = pcl.filter.removeNaNFromPointCloud(cropped)
    local sampled = pcl.filter.randomSample(c, c:size() * self.parameters.random_sampling_portion)

    -- indices based statistical outlier removal
    local denoised = pcl.filter.statisticalOutlierRemoval(
        sampled,  -- input
        self.parameters.outlier_removal.mean_k,       -- meanK
        self.parameters.outlier_removal.standard_deviation_threshold,     -- stddevMulThresh
        nil,      -- indices
        nil,      -- output
        false,    -- negative
        nil,      -- removed_indices
        false     -- keep_organized
    )

    local plane_coefficients = findDominantPlane(self, denoised)

    -- visualize result
    --[[local v = pcl.PCLVisualizer('demo', true)

    x = v:addPlane_Coefficients(plane_coefficients, 0, 0, 0.6)
    v:addPointCloud(denoised, 'test')
    v:addCoordinateSystem()
    v:spin()]]

    local z = x3d.normalize(torch.Tensor({plane_coefficients[1], plane_coefficients[2], plane_coefficients[3]}))
    -- ensure z-axis of plane normal is not pointing away from camera (e.g. height on tray should be positve)
    if z[3] > 0 then
        z = -z
    end
    local y = randOrthogonal(z)
    local x = torch.cross(y, z)

    local origin = x3d.projectOnPlane(denoised:points()[{1,1,{1,3}}]:double(), plane_coefficients:double())

    -- transfom plane into xy-plane (random rotated)
    local T = x3d.identity()
    T[{{1,3},1}] = x
    T[{{1,3},2}] = y
    T[{{1,3},3}] = z
    T[{{1,3},4}] = origin

    cropped:transform(torch.inverse(T):float())

    -- generate image based on z component
    local p = cropped:points()
    local height_map = p[{{},{},3}]:clone()

    local min_val,max_val = self.parameters.height_map.min_z,self.parameters.height_map.max_z

    -- set values outside range to NaN
    height_map[height_map:lt(min_val)] = 0/0
    height_map[height_map:gt(max_val)] = 0/0

    return height_map
end


function HeightAnalysis:createMaskFromContour(contour, width, height)
    local img = torch.ByteTensor(height, width)
    img:zero()
    cv.drawContours{image=img, contours={contour}, contourIdx=-1, color={255, 255, 255}, thickness=-1}
    return img
end


function HeightAnalysis:maskOverlapWithFirst(mask_a, mask_b)
    local a = mask_a:gt(0)
    local area_a = a:sum()
    local b = mask_b:gt(0)
    local mask_and = a:cmul(b)
    local area_and = mask_and:sum()
    local overlap = area_and / area_a
    return overlap
end


function HeightAnalysis:calculateContourHeightMaskOverlap(contour, height_mask)
    local contour_mask = self:createMaskFromContour(contour, height_mask:size(2), height_mask:size(1))
    return self:maskOverlapWithFirst(contour_mask, height_mask)
end



function HeightAnalysis:maskIoU(mask_a, mask_b)
    local a = mask_a:gt(0)
    local b = mask_b:gt(0)
    local mask_and = a:cmul(b)
    local mask_or = (a + b):gt(0)
    local area_and = mask_and:sum()
    local area_or = mask_or:gt(0):sum()
    local iou = area_and / area_or  -- intersection over union
    return iou
end


function HeightAnalysis:generateMask(height_map, low_threshold, high_threshold, scaling)
    scaling = scaling or 255
    local gt = height_map:gt(low_threshold or 0)
    local lt = height_map:lt(high_threshold or math.huge)
    local mask = gt:cmul(lt)
    return mask * scaling
end


function HeightAnalysis:generateColorMap(height_map, style)
    style = style or 'jet'

    local img = height_map:clone()
    colormap:setStyle(style)
    colormap:setSteps(512)

    local map_input = height_map:clone()
    map_input[map_input:ne(map_input)] = 0
    local rgbImg = colormap:convert(map_input)
    local bar = colormap:colorbar(200, 50)
    rgbImg = rgbImg:transpose(1,2):transpose(2,3)
    rgbImg = (rgbImg * 255):byte()

    -- clear out NaN indices
    local nan_indices = height_map:ne(height_map):repeatTensor(3,1,1):permute(2,3,1)
    rgbImg[nan_indices] = 0
    return rgbImg
end


return cap.HeightAnalysis
