# serialcom
-----------

The serialcom library provides accessing serial ports on LINUX systems with real time RTAI compatibility. This library was think to be compatible with process with multiples threads and allows that multiples threads can access the same serial port each time. In that case, these functionalities are implemented to COM1, COM2, COM3 and COM4. Also, this library implements functions to access by semaphores, which allows the same serial port to be access by multiples threads per once.
