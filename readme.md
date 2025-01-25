# VL.Espace

a toolset for spatial applications

**EXPERIMENTAL!** no guarantees are made regarding stability. still needs to be tested heavily...

## Scope

VL.Espace aims on making spatial tracking devices (for now RGBD-cams) more flexible by making them available over IP. This is achieved
* for images: by capturing RGB, depth, etc. frames, compressing them and making them available in a publish-subscribe scenario using VL.IO.NetMQ. a client can subscribe to these publishers, receive and decompress the images and continue working as if the device was available locally
* for body tracking: by acquiring skeleton data and converting them into a device independent generic skeleton datatype it can be serialised and published to the network. 

Abstracing over similar data from different device families should improve the exchangability with regards of the device types and make it easier to build the main application, as they don't have to deal with the specifics of each library.

### Pros and Cons

Splitting the workload of a spatial application to several machines and let them communicate over the network increases scalability and flexibility, but might not be the ideal solution for all scenarios (especially when it needs perfect sync and low latency).


#### Pros

* Data transmission over IP allow for much larger transmission distances compared to a USB connection
* an application might need *m* tracking devices and *n* receiving machines, which is hardly possible over USB
* running a *thin client* (i.e. a computer which is only responsible for data capture from a USB connected tracking device), relieves the main (rendering) machine from workload and reduces library dependencies on those machines

#### Cons
* Data is published in a best effort approach. There are no guarantees regarding transmission latency and that all frames actually will arrive. Application quality will depend heavily on the performance of all devices in the chain (device clients, network, receivers).
* additional computers (thin clients) might be additional points of failure


### Core technology dependencies
* VL.IO.NetMQ for networking
* VL.Skia for JPEG en/decoding of RGB images
* IronSnappy for compression of depth images
* currently supported devices
  * VL.Devices.Azurekinect and VL.Devices.Azurekinect.Body
  * VL.Devices.KinectV2
* VL.OpenCV for camera extrinsics registration

### Additional features
See helppatches for more details:

* contains a few shaders for PointCloud display and depth processing (e.g. a OneEuroFilter compute shader)
* spatial camera calibration using VL.OpenCV

## Known Issues
* Receiver patches suffer from stutters due to Gen2 garbage collections. Would need a hand to identify the culprit. (Sometimes it does run smoothly for a long time though...)
* Kinect2 does not yet support PointCloud displaying. Ideally a ColorDepth image like with Azurekinect would be available for K2 as well. 
  * Not all image types (particularly the K2 version of RGBDepth) are transmitted (yet), which shoud be easy to add though.


## Future Vision

Implementation of additional features is on the wishlist but depending on time availibility

* support of additional RGBD devices (Realsense, Orbbec)
* integration of other tracking device families (LiDAR, XR tracking, hand tracking devices like LEAP)
* completion of skeleton tracking abstraction to create some kind of *unified* skeleton (i.e. a common denominator for different skeleton sources)
* ...


## Thanks
Big shout out to the VL community, most notably ravazquez, catweasel, lasal and the devvvvs for inspiration and code sharing!