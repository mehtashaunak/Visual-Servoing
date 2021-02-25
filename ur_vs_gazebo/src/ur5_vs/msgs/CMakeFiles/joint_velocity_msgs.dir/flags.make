# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# compile CXX with /usr/bin/c++
CXX_FLAGS = -fPIC  

CXX_DEFINES = -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\"ur5_vs\" -Djoint_velocity_msgs_EXPORTS

CXX_INCLUDES = -I/usr/include/gazebo-7 -I/usr/include/sdformat-4.0 -I/usr/include/ignition/math2 -I/home/raina_pc/ur_vs_gazebo/src/devel/include -I/home/raina_pc/ur_vs_gazebo/src/object_detection/include -I/opt/ros/kinetic/include -I/opt/ros/kinetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/opt/ros/kinetic/include/opencv-3.3.1-dev -I/opt/ros/kinetic/include/opencv-3.3.1-dev/opencv -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/ni -I/usr/include/vtk-6.2 -I/usr/include/hdf5/openmpi -I/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent -I/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent/include -I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -I/usr/include/libxml2 -I/usr/include/jsoncpp -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu -I/usr/include/freetype2 -I/usr/include/x86_64-linux-gnu/freetype2 -I/usr/include/tcl -I/home/raina_pc/ur_vs_gazebo/src/ur5_vs/include -I/opt/ros/kinetic/lib/libimage_transport.so -I/opt/ros/kinetic/lib/libcv_bridge.so -I/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1 -I/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1 -I/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1 -I/opt/ros/kinetic/lib/libpcl_ros_filters.so -I/opt/ros/kinetic/lib/libpcl_ros_io.so -I/opt/ros/kinetic/lib/libpcl_ros_tf.so -I/home/raina_pc/ur_vs_gazebo/src/ur5_vs/optimized -I/usr/lib/x86_64-linux-gnu/libpcl_common.so -I/home/raina_pc/ur_vs_gazebo/src/ur5_vs/debug -I/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so -I/usr/lib/x86_64-linux-gnu/libpcl_octree.so -I/usr/lib/x86_64-linux-gnu/libpcl_search.so -I/usr/lib/x86_64-linux-gnu/libpcl_io.so -I/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so -I/usr/lib/x86_64-linux-gnu/libpcl_filters.so -I/usr/lib/x86_64-linux-gnu/libpcl_features.so -I/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so -I/usr/lib/x86_64-linux-gnu/libpcl_surface.so -I/usr/lib/x86_64-linux-gnu/libpcl_registration.so -I/usr/lib/x86_64-linux-gnu/libpcl_recognition.so -I/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so -I/usr/lib/x86_64-linux-gnu/libpcl_visualization.so -I/usr/lib/x86_64-linux-gnu/libpcl_people.so -I/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so -I/usr/lib/x86_64-linux-gnu/libpcl_tracking.so -I/usr/lib/x86_64-linux-gnu/libboost_iostreams.so -I/usr/lib/x86_64-linux-gnu/libboost_serialization.so -I/usr/lib/x86_64-linux-gnu/libqhull.so -I/usr/lib/libOpenNI.so -I/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a -I/usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libz.so -I/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libjpeg.so -I/usr/lib/x86_64-linux-gnu/libpng.so -I/usr/lib/x86_64-linux-gnu/libtiff.so -I/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libfreetype.so -I/usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libnetcdf_c++.so -I/usr/lib/x86_64-linux-gnu/libnetcdf.so -I/usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so -I/usr/lib/x86_64-linux-gnu/libsz.so -I/usr/lib/x86_64-linux-gnu/libm.so -I/usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so -I/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOXML-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libjsoncpp.so -I/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libexpat.so -I/usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.2.so.6.2.0 -I/usr/lib/libgl2ps.so -I/usr/lib/x86_64-linux-gnu/libvtkverdict-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libtheoraenc.so -I/usr/lib/x86_64-linux-gnu/libtheoradec.so -I/usr/lib/x86_64-linux-gnu/libogg.so -I/usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libxml2.so -I/usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0 -I/usr/lib/libvtkWrappingTools-6.2.a -I/usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libsqlite3.so -I/usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeOpenGL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOImport-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOExport-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkVPIC-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.2.so.6.2.0 -I/usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.2.so.6.2.0 -I/opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so -I/opt/ros/kinetic/lib/libnodeletlib.so -I/opt/ros/kinetic/lib/libbondcpp.so -I/usr/lib/x86_64-linux-gnu/libuuid.so -I/usr/lib/x86_64-linux-gnu/libtinyxml2.so -I/opt/ros/kinetic/lib/libclass_loader.so -I/usr/lib/libPocoFoundation.so -I/usr/lib/x86_64-linux-gnu/libdl.so -I/opt/ros/kinetic/lib/libroslib.so -I/opt/ros/kinetic/lib/librospack.so -I/usr/lib/x86_64-linux-gnu/libpython2.7.so -I/opt/ros/kinetic/lib/librosbag.so -I/opt/ros/kinetic/lib/librosbag_storage.so -I/usr/lib/x86_64-linux-gnu/libboost_program_options.so -I/opt/ros/kinetic/lib/libroslz4.so -I/usr/lib/x86_64-linux-gnu/liblz4.so -I/opt/ros/kinetic/lib/libtopic_tools.so -I/opt/ros/kinetic/lib/libtf.so -I/opt/ros/kinetic/lib/librobot_state_publisher_solver.so -I/opt/ros/kinetic/lib/libtf2_ros.so -I/opt/ros/kinetic/lib/libactionlib.so -I/opt/ros/kinetic/lib/libmessage_filters.so -I/opt/ros/kinetic/lib/libtf2.so -I/opt/ros/kinetic/lib/libkdl_parser.so -I/opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0 -I/opt/ros/kinetic/lib/liburdf.so -I/usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so -I/usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so -I/usr/lib/x86_64-linux-gnu/liburdfdom_model.so -I/usr/lib/x86_64-linux-gnu/liburdfdom_world.so -I/usr/lib/x86_64-linux-gnu/libtinyxml.so -I/opt/ros/kinetic/lib/librosconsole_bridge.so -I/opt/ros/kinetic/lib/libroscpp.so -I/usr/lib/x86_64-linux-gnu/libboost_filesystem.so -I/usr/lib/x86_64-linux-gnu/libboost_signals.so -I/opt/ros/kinetic/lib/librosconsole.so -I/opt/ros/kinetic/lib/librosconsole_log4cxx.so -I/opt/ros/kinetic/lib/librosconsole_backend_interface.so -I/usr/lib/x86_64-linux-gnu/liblog4cxx.so -I/usr/lib/x86_64-linux-gnu/libboost_regex.so -I/opt/ros/kinetic/lib/libroscpp_serialization.so -I/opt/ros/kinetic/lib/libxmlrpcpp.so -I/opt/ros/kinetic/lib/librostime.so -I/opt/ros/kinetic/lib/libcpp_common.so -I/usr/lib/x86_64-linux-gnu/libboost_system.so -I/usr/lib/x86_64-linux-gnu/libboost_thread.so -I/usr/lib/x86_64-linux-gnu/libboost_chrono.so -I/usr/lib/x86_64-linux-gnu/libboost_date_time.so -I/usr/lib/x86_64-linux-gnu/libboost_atomic.so -I/usr/lib/x86_64-linux-gnu/libpthread.so -I/usr/lib/x86_64-linux-gnu/libconsole_bridge.so -I/home/raina_pc/ur_vs_gazebo/src/ur5_vs/msgs 
