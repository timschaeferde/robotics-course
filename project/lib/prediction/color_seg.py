from lib.tools.color_operations import *


# segment object using color

def _segment_redball(rgb):
    rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, (0, 120, 70), (10, 255, 255))
    mask2 = cv.inRange(hsv, (170, 120, 70), (180, 255, 255))

    # if len(mask1)>0: cv.imshow('OPENCV - mask1',  mask1)
    # if len(mask2)>0: cv.imshow('OPENCV - mask2',  mask2)
    mask = mask1 + mask2

    # create empty mask
    filtered_mask = np.zeros(mask.shape, np.uint8)

    # if len(filtered_mask)>0: cv.imshow('OPENCV - filtered_mask',  filtered_mask)
    # find contours
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # find largest contour
        largest, idx = 0., None
        for i, c in enumerate(contours):
            # remove noise
            if c.shape[0] < 10:
                continue
            if cv.contourArea(c) > largest:
                largest = cv.contourArea(c)
                idx = i

        cv.drawContours(filtered_mask, contours[idx], -1, (255, 255, 255), -1)
    return filtered_mask


def _image_pointcloud(depth, mask):
    # if len(mask) > 0: cv.imshow('OPENCV - mask1', mask * 255)
    # cv.waitKey(100)
    mask_pixels = np.where(mask > 0)
    pointcloud = np.empty((mask_pixels[0].shape[0], 3))
    pointcloud[:, 0] = mask_pixels[1]  # x pixels
    pointcloud[:, 1] = mask_pixels[0]  # y pixels
    pointcloud[:, 2] = depth[mask_pixels[0], mask_pixels[1]]
    return pointcloud


def _meter_pointcloud(pixel_points, fxfypxpy):
    points = np.empty(np.shape(pixel_points))
    for i, p in enumerate(pixel_points):
        x = p[0]
        y = p[1]
        d = p[2]

        px = fxfypxpy[-2]
        py = fxfypxpy[-1]

        x_ = d * (x - px) / fxfypxpy[0]
        y_ = -d * (y - py) / fxfypxpy[1]
        z_ = -d
        points[i] = [x_, y_, z_]
    return points


def find_ball(ball_color, rgb, depth, fxfypxpy):
    mask = _mask_by_color(
        rgb, ball_color, hsv_tolerance=np.array([10, 200, 200], np.uint8))
    pixel_points = _image_pointcloud(depth, mask)
    obj_points = _meter_pointcloud(pixel_points, fxfypxpy)
    return obj_points, mask


def find_cylinders(rgb, depth, fxfypxpy):
    mask = _mask_by_color(
        rgb, [0, 0, 255], hsv_tolerance=np.array([10, 200, 200], np.uint8))
    pixel_points = _image_pointcloud(depth, mask)
    obj_points = _meter_pointcloud(pixel_points, fxfypxpy)
    return obj_points, mask


def _mask_by_color(rgb, color, hsv_tolerance=np.array([10, 80, 80], np.uint8),
                   hsv_min=np.array([0, 20, 20], np.uint8), hsv_max=np.array([360 / 2, 255, 255], np.uint8)):
    # BE CAREFUL all colors in rai are however in bgr not rgb!
    # rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
    # color = cv.cvtColor(np.asarray([[color]]).astype(np.uint8), cv.COLOR_BGR2RGB)[0][0]

    # if len(rgb) > 0: cv.imshow('OPENCV - rgb2', rgb)
    # cv.imshow('OPENCV - rgb2', np.array([[color]],np.uint8))
    # cv.waitKey(10)

    hsv_img = rgb2hsv(rgb)
    hsv_color = single_rgb2hsv(color)
    # print(color_hsv)

    # make sure to not get below 0
    hsv_low = np.maximum(hsv_color, hsv_min + hsv_tolerance) - hsv_tolerance

    # make sure to not get higher than 180 or 225
    hsv_high = np.minimum(hsv_color, hsv_max - hsv_tolerance) + hsv_tolerance

    # print(hsv_low, hsv_high)

    mask = cv.inRange(hsv_img, hsv_low, hsv_high)
    # if len(mask)>0: cv.imshow('OPENCV - mask',  mask)

    # create empty mask
    filtered_mask = np.zeros(mask.shape, np.uint8)

    # find contours
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    cv.drawContours(filtered_mask, contours, -1, (255, 255, 255), -1)
    # if len(filtered_mask)>0: cv.imshow('OPENCV - filtered_mask',  filtered_mask)
    return filtered_mask


def get_mask_by_color(rgb, color):
    # hue tolerance should be 1
    return _mask_by_color(rgb, color, np.array([1, 200, 200], np.uint8))


def get_points_from_mask(mask, depth, fxfypxpy):
    pixel_points = _image_pointcloud(depth, mask)
    obj_points = _meter_pointcloud(pixel_points, fxfypxpy)
    return obj_points

###########################################################

# def _segment_depth(depth, depth0):
#     depth_mask = cv.absdiff(depth, depth0)
#     ret, mask = cv.threshold(depth_mask, 0, 5, cv.THRESH_BINARY)
#     mask = mask * 255
#     return mask.astype("uint8")

# # combining both color & depth, requires initial depth image
# def segment_object(rgb, depth, depth0):
#     mask_rgb = _segment_redball(rgb)
#     mask_depth = _segment_depth(depth, depth0)
#     mask = cv.bitwise_and(mask_rgb, mask_depth)
#     return mask
