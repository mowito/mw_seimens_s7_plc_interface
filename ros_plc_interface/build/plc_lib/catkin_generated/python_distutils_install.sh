#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/src/plc_lib"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/install/lib/python2.7/dist-packages:/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/build" \
    "/usr/bin/python2" \
    "/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/src/plc_lib/setup.py" \
     \
    build --build-base "/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/build/plc_lib" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/install" --install-scripts="/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/install/bin"
