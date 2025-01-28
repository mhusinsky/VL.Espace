# Related projects

## RGB-D frameworks
### RGB-D
* Sensor Stream Pipe
  * https://github.com/moetsi/Sensor-Stream-Pipe
    * Framework with SAAS
      * https://www.moetsi.com/
* XRCAP (unfinished)
  * https://github.com/catid/xrcap/
* Depthkit
  * https://www.depthkit.tv/
* Mirevi Motion Hub (HS Düsseldorf)
  * https://mirevi.de/publications/motionhub/
  * https://github.com/Mirevi/MotionHub


## image compression
* Depth image specific
  * https://github.com/catid/Zdepth
    * used in moetsi Sensor Stream Pipe
    * based on MS
  * https://github.com/catid/Zdepth
    * lossy version of Zdepth
  * MS Fast Lossless Image compression
    * https://www.microsoft.com/en-us/research/publication/fast-lossless-depth-image-compression/
* Generic Compression Methods
  * K4os.Compression: Ships with VVVV
    * https://github.com/K4os/compression
  * -> compare to Snappy
* JPEG Compression
  * currently using Skia compressor
  Alternatives?
    * https://github.com/subalterngames/fast_image_encoder
    * ImageSharp. (Is a dependency of Azurekinect package in an old version. Should this be cut out for better version management and usage in different packages?)
    * https://libjpeg-turbo.org/


## Motion Tracking
### body

* XBOX Kinect
* Kinect2
* AzureKinect
* Stereolab ZED
  * https://www.stereolabs.com/docs/body-tracking/
  * BODY_18, BODY_34, BODY_38
* OAK-D

* Mediapipe
* Sony mocopi
  * https://www.sony.net/Products/mocopi-dev/en/documents/Home/Aboutmocopi.html
* Unity Humanoid Rig
  * Not tracking but the unity standard Avatar rig setup
* X3D H-Anim
  * https://x3dgraphics.com/slidesets/X3dForAdvancedModeling/HumanoidAnimation.pdf

### hands

* OpenXR / Quest
* Mediapipe
* LeapMotion/Ultraleap