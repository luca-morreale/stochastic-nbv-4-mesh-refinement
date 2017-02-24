/*
  cobblestone-street height_field generator
  *
  Jaime Vives Piqueres, ?(C), 1999.
*/

#version 3.0
#include "colors.inc"
#include "textures.inc"


#declare t_brick=
texture{
 pigment{
  granite
  turbulence 0.05
  color_map{
   [0.0 White*.95]
   [1.0 White]
  }
  scale .5
 }
 finish{ambient 1 diffuse 0}
}
texture{
 pigment{
  dents
  turbulence 0.3
  color_map{
   [0.0 White filter 0.8]
   [0.1 White*.95 filter 0.9]
   [0.2 Clear]
   [1.0 Clear]
  }
  scale .03
 }
 finish{ambient 1 diffuse 0}
}
texture{
 pigment{
  spotted
  turbulence 0.6
  color_map{
   [0.0 White*.8 filter 0.7]
   [0.6 White*.95 filter 0.9]
   [0.7 Clear]
   [1.0 Clear]
  }
  scale .1
 }
 finish{ambient 1 diffuse 0}
}

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
    #declare kk=rand(r_brick)
    object{
     one_brick
     texture{t_brick
      scale (row_height+col_width)
      translate <num_cols*col_width*rand(r_brick),num_cols*col_width*rand(r_brick),0>
     }
     texture{pigment{White*(.6+.4*rand(r_brick)) filter 0.9} finish{Luminous}}
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


plane{
 z,0
 texture{t_brick scale <2,1,1>
  scale (row_height+col_width)
  translate <num_cols*col_width*2*rand(r_brick),num_cols*col_width*2*rand(r_brick),0>
 }
 texture{
  pigment{White*.9 filter 0.9}
 }
}

camera{
 location <0.01,-0.01,-(num_cols*col_width+num_rows*row_height)>
 direction 3*z
 look_at <-.001,0.01,5>
 orthographic
}

