import unittest
from types import SimpleNamespace as S
import numpy as np
from fr3c_camera.rgbd import (image_array, depth_metres, map_color_pixel_to_depth,
                              project_patch, validate_pair, project_color_pixel)
from fr3c_camera.preprocess import (colorize_adaptive_depth, colorize_metric_depth,
                                    depth_discontinuity_mask, roi_depth_statistics,
                                    temporal_consensus_depth, valid_depth_mask)


def info():
    return S(k=[100.,0.,1.,0.,100.,1.,0.,0.,1.],d=[0.]*5,distortion_model='plumb_bob',
             width=3,height=3,header=S(frame_id='camera_color_optical_frame',stamp=S(sec=100,nanosec=0)),
             binning_x=0,binning_y=0,roi=S(x_offset=0,y_offset=0))


def image(encoding='16UC1', big=False):
    a=np.full((3,3),1000,dtype='>u2' if big else '<u2')
    data=b''.join(row.tobytes()+b'\x00\x00' for row in a)
    return S(width=3,height=3,step=8,encoding=encoding,is_bigendian=big,data=data,header=info().header)


class ProjectionTests(unittest.TestCase):
    def test_padded_big_endian_mm(self):
        self.assertTrue(np.allclose(depth_metres(image(big=True)),1.))
        self.assertEqual(image_array(image()).shape,(3,3))

    def test_ray_and_units(self):
        xyz,_,_=project_patch(np.ones((3,3)),info(),2.,1.,radius=0)
        np.testing.assert_allclose(xyz,[.01,0,1])

    def test_invalid_and_edge_depth(self):
        for a in (np.zeros((3,3)),np.full((3,3),np.nan),np.array([[1.,1.,1.],[1.,2.,2.],[2.,2.,2.]])):
            with self.assertRaises(ValueError): project_patch(a,info(),1.,1.)

    def test_distorted_pixel(self):
        i=info();i.d=[.1,0,0,0,0]
        i.k=[100.,0.,0.,0.,100.,0.,0.,0.,1.]
        # x=0.5 maps to distorted x=0.5125 under k1=0.1.
        xyz,_,_=project_patch(np.ones((100,100)),i,51.25,0.,radius=0)
        self.assertAlmostEqual(xyz[0],.5,places=5)

    def test_contract_rejects_stale_skew_frames_intrinsics(self):
        for modification in ('stale','skew','frame','intrinsic'):
            ci,di=info(),info(); c,d=image('rgb8'),image()
            if modification=='stale': now=101.
            else: now=100.1
            if modification=='skew':d.header.stamp.nanosec=90000000
            if modification=='frame':d.header.frame_id='depth_frame'
            if modification=='intrinsic':di.k[0]=101.
            with self.assertRaises(ValueError): validate_pair(c,d,ci,di,now)

    def test_raw_color_to_rectified_depth(self):
        ci,di=info(),info()
        for i in (ci,di):
            i.width=i.height=100
            i.k=[100.,0.,0.,0.,100.,0.,0.,0.,1.]
        ci.d=[.1,0,0,0,0]
        depth=np.zeros((100,100));depth[0,50]=1.
        np.testing.assert_allclose(map_color_pixel_to_depth(ci,di,51.25,0.),[50.,0.],atol=1e-6)
        xyz,_,_=project_color_pixel(depth,ci,di,51.25,0.,radius=0)
        np.testing.assert_allclose(xyz,[.5,0.,1.],atol=1e-6)

    def test_valid_contract(self):
        c,d=image('rgb8'),image()
        np.testing.assert_allclose(validate_pair(c,d,info(),info(),100.1),1.)


class PreprocessTests(unittest.TestCase):
    def test_fixed_metric_color_keeps_invalid_black(self):
        depth=np.array([[0.,.3,.75,1.2,1.3,np.nan]],dtype=np.float32)
        color,valid=colorize_metric_depth(depth,.3,1.2)
        np.testing.assert_array_equal(valid,[[False,True,True,True,False,False]])
        self.assertTrue(np.all(color[0,[0,4,5]]==0))

    def test_windows_adaptive_color_keeps_only_zero_invalid(self):
        depth=np.array([[0.,.2,.4,.8,1.6,np.nan]],dtype=np.float32)
        color,valid,scale=colorize_adaptive_depth(depth)
        np.testing.assert_array_equal(valid,[[False,True,True,True,True,False]])
        self.assertTrue(np.all(color[0,[0,5]]==0))
        self.assertGreater(scale['high_mm'],scale['low_mm'])

    def test_temporal_consensus_never_fills_latest_hole(self):
        frames=np.full((5,3,3),.75,dtype=np.float32)
        frames[0,1,1]=.755
        frames[-1,0,0]=0.
        filtered,stable,report=temporal_consensus_depth(frames,.3,1.2,.6,.015)
        self.assertFalse(stable[0,0])
        self.assertTrue(np.isnan(filtered[0,0]))
        self.assertTrue(stable[1,1])
        self.assertAlmostEqual(float(filtered[1,1]),.75)
        self.assertEqual(report['required_valid_frames'],3)

    def test_temporal_consensus_rejects_motion(self):
        frames=np.array([np.full((2,2),z,dtype=np.float32)
                         for z in (.70,.72,.74,.76,.78)])
        filtered,stable,_=temporal_consensus_depth(frames,.3,1.2,.6,.015)
        self.assertFalse(stable.any())
        self.assertTrue(np.isnan(filtered).all())

    def test_roi_statistics_and_valid_mask(self):
        depth=np.full((5,5),.6,dtype=np.float32)
        depth[1,1]=0.;depth[2,2]=.7
        result=roi_depth_statistics(depth,2,2,1,.3,1.2)
        self.assertEqual(result['total_pixels'],9)
        self.assertEqual(result['valid_pixels'],8)
        self.assertAlmostEqual(result['valid_fraction'],8/9)
        self.assertAlmostEqual(result['median_mm'],600.)
        self.assertEqual(valid_depth_mask(depth,.3,1.2).shape,(5,5))

    def test_depth_edges_ignore_hole_boundaries(self):
        depth=np.full((9,9),.6,dtype=np.float32)
        depth[:,5:]=.9
        depth[2,2]=0.
        edges=depth_discontinuity_mask(depth,.3,1.2)
        self.assertTrue(edges[:,4:6].any())
        self.assertFalse(edges[1:4,1:4].any())

if __name__=='__main__':unittest.main()
