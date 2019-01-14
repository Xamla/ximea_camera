local torch = require 'torch'
local pcl = require 'pcl'
local cap = require 'ximea_nodes'

local CropBoxStatistics = torch.class('ximea_nodes.CropBoxStatistics', cap)

local function computeCropBoxStatistics(cloud, crop_box)
    print('compute for crop box')
    local box_min =
        torch.FloatTensor(
        {
            crop_box.min_x,
            crop_box.min_y,
            crop_box.min_z
        }
    )

    local box_max =
        torch.FloatTensor(
        {
            crop_box.max_x,
            crop_box.max_y,
            crop_box.max_z
        }
    )

    print('box min:', box_min)
    print('box max:', box_max)

    local d = torch.tic()
    local inlier_indices =
        pcl.filter.cropBox(
        cloud, -- input
        box_min, -- min
        box_max, -- max
        nil, -- rotation
        nil, -- translation
        nil, -- transform
        nil, -- indices
        pcl.Indices(), -- output
        false, -- negative
        nil, -- removed_indices
        true -- keep_organized
    )
    print('processing time crop:', torch.toc(d))

    local cropped_cloud = pcl.filter.extractIndices(cloud, inlier_indices)

    local centroid = cropped_cloud:compute3DCentroid()
    local covariance = cropped_cloud:computeCovarianceMatrix(centroid)

    return inlier_indices:size(), centroid, covariance
end

function CropBoxStatistics.getDefaultParameters()
    return {
        -- first filter run on input data
        crop_box = {
            min = {-0.5, -0.5, 0.4}, -- min point for crop box
            max = {0.5, 0.5, 0.8} -- max point for crop box
        },
        -- statistical outlier removal
        outlier_removal = {
            mean_k = 16,
            standard_deviation_threshold = 0.01
        }
    }
end

function CropBoxStatistics:__init()
    self.parameters = CropBoxStatistics.getDefaultParameters()
end

function CropBoxStatistics:setDefaultCrop(crop_box)
    local box_min =
        torch.FloatTensor(
        {
            crop_box.min_x,
            crop_box.min_y,
            crop_box.min_z
        }
    )

    local box_max =
        torch.FloatTensor(
        {
            crop_box.max_x,
            crop_box.max_y,
            crop_box.max_z
        }
    )

    self.parameters.crop_box.min = box_min
    self.parameters.crop_box.max = box_max
end

function CropBoxStatistics:run(cloud, pose, crop_boxes)
    -- crop box
    print('default crop box min:', self.parameters.crop_box.min)
    print('default crop box max:', self.parameters.crop_box.max)

    local d = torch.tic()
    local cropped =
        pcl.filter.cropBox(
        cloud, -- input
        self.parameters.crop_box.min, -- min
        self.parameters.crop_box.max, -- max
        nil, -- rotation
        nil, -- translation
        torch.eye(4), -- transform
        nil, -- indices
        nil, -- output
        false, -- negative
        nil, -- removed_indices
        true -- keep_organized
    )
    print('processing time default crop:', torch.toc(d))

    -- remove nan indices
    local c = pcl.filter.removeNaNFromPointCloud(cropped)
    local sampled = pcl.filter.randomSample(c, c:size() * 0.5)

    -- statistical outlier removal
    d = torch.tic()
    local denoised =
        pcl.filter.statisticalOutlierRemoval(
        sampled, -- input
        self.parameters.outlier_removal.mean_k, -- meanK
        self.parameters.outlier_removal.standard_deviation_threshold, -- stddevMulThresh
        nil, -- indices
        nil, -- output
        false, -- negative
        nil, -- removed_indices
        true -- keep_organized
    )
    print('processing time denoising:', torch.toc(d))

    d = torch.tic()
    local transformed_denoised = denoised:transform(pose)
    print('processing time pointcloud transform:', torch.toc(d))

    local point_counts = {}
    local centroids = {}
    local covariances = {}

    for i, crop_box in ipairs(crop_boxes) do
        local point_count, centroid, covariance = computeCropBoxStatistics(transformed_denoised, crop_box)
        point_counts[i] = point_count
        centroids[i] = centroid
        covariances[i] = covariance
    end

    return point_counts, centroids, covariances
end

return cap.CropBoxStatistics
