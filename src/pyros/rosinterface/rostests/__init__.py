# Because rospy :
# - expects OS state to be very special
# - modify global python interpreter state when using node_init ( which is required to receive message on topics )
# => Communication features are only testable with rostest.
# However, running them one by one with nosetest works (One run, one test module, one process).
# This allows easy debugging ( even during start / stop sequences )
# Therefore, tests are executable so that nosetests doesn't auto detect them.
