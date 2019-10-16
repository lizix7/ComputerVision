# OpenCV Notes

## 1. OpenCV Bitwise Logic Operation
  
  If each pixel is only 0 or 1, then logic operation is easy to understand.
  If each pixel is a decimal number, then logic operation is hard to understand.
  
  Because for each colored image, it has RGB 3 channels. And the range is from 0 to 255. 
  For an example: 
  
  `A = (105, 115, 102) B = (73, 155, 150)`
  
  `C = A & B = (73, 19, 6)`
  
  It is hard to understand if you try to think in the decimal way. 
  What OpenCV does here is that:
  
    1. Transfer both decimal numbers into binary numbers
    
    2. Do bitwise logic operation
    
    3. Transfer the binary result back to decimal number
  
  Now let's do bitwise logic AND to A and B:
  
    105 & 73 = 73
    115 & 155 = 19
    102 & 150 = 6
   All the above can be verified by the [bitwise calculator](https://miniwebtool.com/bitwise-calculator/?data_type=10&number1=102&number2=150&operator=AND).

  It is still hard to understand, because it is not intuitive. 
  
  If B is always white, B = (255,255,255).
  
    A & B = A
    A | B = B
    A ^ B = B - A 
    
    12 ^ 255 = 243
  
  If B is always black, B = (0,0,0).
  
    A & B = B
    A | B = A
    A ^ B = A
    
  If both A and B belong in (0,255), A ^ B has no meaning. Only when A is very close to B, A ^ B will give you a very small result, which represents a dark color. This is also the reason that the bitwise_xor result of two similar images has the background color as black. When A is very different to B, A ^ B will give you a very large result, which represents a bright color. This is the reason that the defects look really bright in the bitwise_xor result image.
