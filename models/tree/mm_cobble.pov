/*
  Cobblestone-street material_map generation scene :
  note: convert later tga to gif 256 colors to act as material_map
        over hf_wbrk.tga
  *
  Jaime Vives Piqueres, ?(C), 1999.

*/

#version 3.0
global_settings { assumed_gamma 2.2 hf_gray_16 }
#include "colors.inc"
#include "textures.inc"

#declare r_brick=seed(43);
#declare num_rows=27*2;
#declare row_height=20;
#declare num_cols=11*2;
#declare col_width=50;
#declare mortar_thick=4;
#declare one_brick=
superellipsoid{
 <.1,.1>
 rotate 90*x
 scale .5
 scale <col_width-mortar_thick,row_height-mortar_thick,1>
 translate <col_width*.5,row_height*.5,0>
}
/*
box{
 -.5,.5
 scale <col_width-mortar_thick,row_height-mortar_thick,1>
 translate <col_width*.5,row_height*.5,0>
}
*/
#declare cnt_rows=1;
union{
 #while (cnt_rows<=num_rows)
  #declare cnt_cols=1;
  #while (cnt_cols<=num_cols)
    object{
     one_brick
     texture{
      pigment{rgb (1+int(5*rand(r_brick)))/6}
      finish{Luminous}
      translate <num_cols*col_width*rand(r_brick),num_cols*col_width*rand(r_brick),0>
     }
     #declare kk=rand(r_brick);
     translate <-mortar_thick*.2+mortar_thick*.4*rand(r_brick),
                -mortar_thick*.2+mortar_thick*.4*rand(r_brick),
                0>
     translate <(cnt_cols-1)*col_width,(cnt_rows-1)*row_height,0>
     #if (mod(cnt_rows,2)=0)
       translate col_width*.5*x
     #end
    }
    #declare cnt_cols=cnt_cols+1;
  #end
  #declare cnt_rows=cnt_rows+1;
 #end
 translate <-num_cols*col_width*.5,-num_rows*row_height*.5,0>
}

camera{
 location <0.01,-0.01,-(num_cols*col_width+num_rows*row_height)>
 direction 3*z
 look_at <-.001,0.01,5>
 orthographic
}

