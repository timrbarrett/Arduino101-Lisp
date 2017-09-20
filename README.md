# Arduino101-Lisp
BLE Based Lisp REPL

Stage 1 : ASCII tx and rx working between Arduino101 and nRF uart v2.0 Android program DONE (Sept 19 2017)
          With freememory, stack and heap memory reported

Stage 2 : Simple REPL working over BLE, with integers, strings, symbols, basic functions
          quote, cons, cond, car, cdr, plus, minus, mult, div, mod
          With reporting on use of memory devoted to lisp
          With interrupts able to initate lisp execution code in same environment
          TestSuite in lisp. Rabbit?

Stage 3 : Basic analog and digital reading and writing

Stage 4 : Integrating IMU (step detection), Curie Neurons, RTC
          Track how long things take.

