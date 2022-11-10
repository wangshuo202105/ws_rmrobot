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

echo_and_run cd "/home/ws/ws_rmrobot/src/easy_handeye/rqt_easy_handeye"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ws/ws_rmrobot/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ws/ws_rmrobot/install/lib/python2.7/dist-packages:/home/ws/ws_rmrobot/build/rqt_easy_handeye/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ws/ws_rmrobot/build/rqt_easy_handeye" \
    "/usr/bin/python2" \
    "/home/ws/ws_rmrobot/src/easy_handeye/rqt_easy_handeye/setup.py" \
    egg_info --egg-base /home/ws/ws_rmrobot/build/rqt_easy_handeye \
    build --build-base "/home/ws/ws_rmrobot/build/rqt_easy_handeye" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ws/ws_rmrobot/install" --install-scripts="/home/ws/ws_rmrobot/install/bin"
