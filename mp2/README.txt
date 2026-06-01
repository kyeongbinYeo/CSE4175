Computer Networks - Machine Problem 2
=====================================

This package contains everything you need to develop your MP2 router.

Files
-----
  mp2.pdf            - Assignment specification. READ THIS FIRST.
  netsim2.h          - API header to #include in your router code.
  netsim2_lib.cc     - Simulation runtime; compile together with your code.
  val1.scn ~ val4.scn  - 4 public validation scenarios.

What to write
-------------
A single C/C++ source file named:
    router_<your_student_id>.cc        (e.g. router_20200001.cc)

It must implement the callbacks declared in netsim2.h and may call the
helper functions (send_control, get_now, schedule_wakeup) declared there.

How to compile and run
----------------------
    g++ -O2 -o router_20200001 router_20200001.cc netsim2_lib.cc
    ./router_20200001 --scenario val1.scn

The summary is printed to stderr at the end of the run.

What to submit
--------------
ONLY router_<your_id>.cc. Do not submit netsim2.h, netsim2_lib.cc, or the
.scn files. The grading server has its own copies (plus hidden test
scenarios).

For full details (cost formula, scoring, rules) see mp2.pdf.
