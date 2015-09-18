# master
Eye Tracking for point gray cameras

REQUIRES openCV > 3.0; TBB; BOOST; COMEDI...


Build openCV with CXXFLAGS="-ffast-math -march=native" CFLAGS=$CXXFLAGS cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..


cmake .

make



INITIAL COMMIT NOT READY FOR PRODUCTION !!!


Thanks to Michael Rabadi

Thanks to Bijan Paeseran for the cameras

Thanks to Lech Swirski for the off angle pupil tracking algorithm

Thanks to Dongheng Li for the starburst algorithm

Świrski, L., Bulling, A., & Dodgson, N. (2012, March). Robust real-time pupil tracking in highly off-axis images. In Proceedings of the Symposium on Eye Tracking Research and Applications (pp. 173-176). ACM.

Dongheng, L., & Parkhurst, J. D. (2005, September). Starburst: A robust algorithm for video-based eye tracking. In Proceedings of the IEEE Vision for Human-Computer Interaction Workshop.
