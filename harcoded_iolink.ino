/*
	Copyright (C) 2021 unref-ptr
    This file is part of Hardcoded IO-Link.

    Hardcoded IO-Link is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Hardcoded IO-Link is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Hardcoded IO-Link.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
	IO-Link Example for 
	using the hardcoded IO-Link.
	This is a proof of concept and in no means
	complies completely with the spec.
*/
#include <stdint.h>
#include "hard_code_iolink.h"

hardcode_iolink simple_iolink_device;

uint8_t pdOut[2];

void setup() {
  //Specifiy Serial HW Object for use with IO-Link
  simple_iolink_device.init(Serial);
  Serial.begin(IO_LINK_COM2, SERIAL_8E1);
}

void loop() {

  simple_iolink_device.task();
  bool pdOut_ok=simple_iolink_device.getPDOut(pdOut);
  if(pdOut_ok)
  {
    //dosomething with pdOut
  }
}
