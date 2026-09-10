"""RGB-D validation and projection. Units: Image 16UC1 mm; output metres."""
import cv2
import numpy as np


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_array(msg):
    formats = {'16UC1': ('u2', 1), '32FC1': ('f4', 1),
               'rgb8': ('u1', 3), 'bgr8': ('u1', 3)}
    if msg.encoding not in formats:
        raise ValueError('unsupported encoding: ' + msg.encoding)
    kind, channels = formats[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.height <= 0 or msg.width <= 0 or msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
        raise ValueError('invalid image dimensions/stride/buffer')
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (msg.step, dtype.itemsize) if channels == 1 else (msg.step, channels * dtype.itemsize, dtype.itemsize)
    return np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)


def depth_metres(msg):
    if msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError('depth must be 16UC1 or 32FC1')
    return image_array(msg).astype(np.float32) * (0.001 if msg.encoding == '16UC1' else 1.0)


def validate_pair(color, depth, color_info, depth_info, now, max_age=0.5, max_skew=0.06):
    messages = (color, depth, color_info, depth_info)
    stamps = [stamp_seconds(m) for m in messages]
    if min(stamps) <= 0 or any(now - s > max_age or s - now > 0.05 for s in stamps):
        raise ValueError('stale_or_future_timestamp')
    if max(stamps) - min(stamps) > max_skew:
        raise ValueError('rgbd_timestamp_skew')
    frames = [m.header.frame_id for m in messages]
    if not frames[0] or len(set(frames)) != 1:
        raise ValueError('unaligned_optical_frames')
    if len({(m.width, m.height) for m in messages}) != 1:
        raise ValueError('unaligned_dimensions')
    if color.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('unsupported_color_encoding')
    for info in (color_info, depth_info):
        k = np.asarray(info.k)
        if not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0 or k[8] != 1:
            raise ValueError('invalid_intrinsics')
        if not np.isfinite(info.d).all():
            raise ValueError('invalid_distortion')
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            raise ValueError('unsupported_binning_or_roi')
    if not np.allclose(color_info.k, depth_info.k, rtol=1e-5, atol=1e-5):
        raise ValueError('unaligned_intrinsics')
    return depth_metres(depth)


def project_patch(depth_m, info, u, v, radius=2, min_depth=0.1, max_depth=3.0, min_valid=0.6):
    h, w = depth_m.shape
    if not (np.isfinite([u, v]).all() and 0 <= u < w and 0 <= v < h):
        raise ValueError('pixel_out_of_image')
    if radius < 0 or not 0 < min_valid <= 1 or not 0 < min_depth < max_depth:
        raise ValueError('invalid_projection_parameters')
    x, y = int(round(u)), int(round(v))
    x, y = min(x, w-1), min(y, h-1)
    patch = depth_m[max(0,y-radius):min(h,y+radius+1), max(0,x-radius):min(w,x+radius+1)]
    valid = np.isfinite(patch) & (patch >= min_depth) & (patch <= max_depth)
    fraction = float(valid.mean())
    if fraction < min_valid:
        raise ValueError('insufficient_valid_depth')
    values = patch[valid]
    z = float(np.median(values))
    # Reject neighbourhoods straddling a foreground/background edge.
    spread = float(np.percentile(values,90) - np.percentile(values,10))
    if spread > max(0.02, z * 0.03):
        raise ValueError('depth_discontinuity')
    k = np.asarray(info.k, dtype=np.float64).reshape(3,3)
    d = np.asarray(info.d, dtype=np.float64)
    pixel = np.array([[[u,v]]], dtype=np.float64)
    if info.distortion_model in ('plumb_bob', 'rational_polynomial'):
        xy = cv2.undistortPoints(pixel, k, d).reshape(2)
    elif info.distortion_model == 'equidistant':
        if len(d) != 4: raise ValueError('invalid_fisheye_distortion')
        xy = cv2.fisheye.undistortPoints(pixel, k, d).reshape(2)
    elif len(d) == 0 or np.all(d == 0):
        xy = np.array([(u-k[0,2])/k[0,0], (v-k[1,2])/k[1,1]])
    else:
        raise ValueError('unsupported_distortion_model')
    xyz = np.array([xy[0]*z, xy[1]*z, z])
    if not np.isfinite(xyz).all(): raise ValueError('nonfinite_projection')
    return xyz.tolist(), fraction, spread


def map_color_pixel_to_depth(color_info, depth_info, u, v):
    """Map a raw RGB pixel into the registered depth image's pixel grid."""
    if not (0 <= u < color_info.width and 0 <= v < color_info.height):
        raise ValueError('pixel_out_of_image')
    k = np.asarray(color_info.k,dtype=np.float64).reshape(3,3)
    d = np.asarray(color_info.d,dtype=np.float64)
    pixel = np.array([[[u,v]]],dtype=np.float64)
    if color_info.distortion_model in ('plumb_bob','rational_polynomial'):
        ray = cv2.undistortPoints(pixel,k,d).reshape(2)
    elif color_info.distortion_model == 'equidistant':
        ray = cv2.fisheye.undistortPoints(pixel,k,d).reshape(2)
    elif len(d)==0 or np.all(d==0):
        ray = np.array([(u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1]])
    else: raise ValueError('unsupported_color_distortion_model')
    kd = np.asarray(depth_info.k,dtype=np.float64).reshape(3,3)
    dd = np.asarray(depth_info.d,dtype=np.float64)
    if depth_info.distortion_model in ('plumb_bob','rational_polynomial'):
        mapped,_ = cv2.projectPoints(np.array([[ray[0],ray[1],1.]]),np.zeros(3),np.zeros(3),kd,dd)
    elif depth_info.distortion_model=='equidistant':
        mapped = cv2.fisheye.distortPoints(ray.reshape(1,1,2),kd,dd)
    elif len(dd)==0 or np.all(dd==0):
        mapped = np.array([ray[0]*kd[0,0]+kd[0,2],ray[1]*kd[1,1]+kd[1,2]])
    else: raise ValueError('unsupported_depth_distortion_model')
    return tuple(float(value) for value in mapped.reshape(2))


def project_color_pixel(depth_m, color_info, depth_info, u, v, **kwargs):
    """Map a raw RGB pixel into the registered depth image's distortion model.

    This device reports nonzero RGB distortion but zero D2C depth distortion.
    Matching K/frame alone is therefore insufficient to use identical pixels.
    Assumes driver D2C places depth in the color optical frame (validated above).
    """
    ud,vd = map_color_pixel_to_depth(color_info, depth_info, u, v)
    return project_patch(depth_m,depth_info,float(ud),float(vd),**kwargs)
