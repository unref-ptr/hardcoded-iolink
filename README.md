
## Hard Coded IO-Link:

IO-Link Device for use with the Arduino platform. As the name imples, the IO-Link device stack is hardcoded and does not comply with the IO-Link specification. In general it can be used to test the IO-Link communication protocol and learn how it works. You can try using this softwware with an IO-Link Transciever (e.g. TIOL-111).  It implements the most simplest way to communicate and recieve data from an IO-Link Master (PD Out). The pinout connection is found on the header file. In general it requires 4 pins

* Wakeup (Digital Input with falling edge interrupt) <br>
* TX (Transmission from the Arduino to the IOL Device transciever)<br>
* RX (Reception of data from the IO-L Master via the IOL Device transciever)<br>
* Enable (Digital output, when HIGH it allows to send data to the transciever otherwise its in reception mode or waiting for the Wakuep signal)<br>

Note: only tested with TIOL-111. IF using any other transciever the functionality of the pins may need to be modified accordingly.

## Limitations

* The IOL Device does not include use of Parameters, Events or DS. 
* IF the device comm is interrupted, it requires to reset the SW does not implement the deivce mode handler.
* Blocking the main task with any other task might interrupt the program, as IO-Link expects a maxmimum device response time of 10/COM where COM is any of the baudrates for IO-Link.




#### Improvements:

You are welcome to improve the software and include features that are not part of this implementation by forking and creating a merge request.

#### Further information:

Just did this as an experiment to check how much can be achievd without implementing the complete spec. More of a simple and lightweight stack. <br>
For more informaiton RTFM (i.e. the IO-Link specification ) ;) <br> 

