#include "MS4525.h"

/*
Not a ton of info on MS4525 - datasheet has no register map.

14-bit output pressure, 11-bit output temperature.

Update time: 0.5ms

Output is proportional to difference between Port 1 and Port 2.  Positive when Port 1 > Port 2, and 50% of total counts at Port 1 = Port 2.
Output type of this model is "Output Type A", 10%-90% (as opposed to B, which is 5%-95% - not sure what this means)

Max I2C freq: 400 kHz, min is 100 kHz

Note: 
Response will go nonlinear below 14 deg F.  Can be corrected for, but not implemented.


Datasheet:
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS4525DO%7FB2%7Fpdf%7FEnglish%7FENG_DS_MS4525DO_B2.pdf%7FCAT-BLPS0002
(PX4AIRSPEEDV1.2 sensor is the 1 psi, differential pressure version - 001D)

*/