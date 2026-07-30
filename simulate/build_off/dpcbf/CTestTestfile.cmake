# CMake generated Testfile for 
# Source directory: /home/kwan/unitree_rl_mjlab_/dpcbf
# Build directory: /home/kwan/unitree_rl_mjlab_/simulate/build_off/dpcbf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(dpcbf_safety_filter_test "/home/kwan/unitree_rl_mjlab_/simulate/build_off/dpcbf/dpcbf_safety_filter_test" "/home/kwan/unitree_rl_mjlab_/dpcbf/config/dpcbf_config.yaml")
set_tests_properties(dpcbf_safety_filter_test PROPERTIES  _BACKTRACE_TRIPLES "/home/kwan/unitree_rl_mjlab_/dpcbf/CMakeLists.txt;69;add_test;/home/kwan/unitree_rl_mjlab_/dpcbf/CMakeLists.txt;0;")
subdirs("../_deps/abseil-cpp-build")
subdirs("../_deps/osqp-cpp-build")
