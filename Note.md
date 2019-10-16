# OpenCV Notes

1. OpenCV Bitwise Logic Operation
  
  If each pixel is only 0 or 1, then logic operation is easy to understand.
  If each pixel is a decimal number, then logic operation is hard to understand.
  
  Because for each colored image, it has RGB, 3 channels. And the range is from 0 to 255. 
  `A = (105, 115, 102) B = (73, 155, 150)`
  `C = A & B = (73,  19,   6)`
