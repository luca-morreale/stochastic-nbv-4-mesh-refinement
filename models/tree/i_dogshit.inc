// *** dog shit ***
#declare t_shit=texture{
 pigment{
  granite
  color_map{
   [0.0 DarkBrown*.75]
   [1.0 DarkTan*.75]
  }
 }
 scale 2
 normal{wood}
}
#declare shit =
julia_fractal {
 <-0.1211,0.0,-0.45686,-0.03341>
 quaternion
 cube
 max_iteration 8
 precision 15
 texture{t_shit}
 scale 6
 rotate <90,0,90>
}
