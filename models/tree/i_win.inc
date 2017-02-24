/* -------------------------------------------------------------------------
   "I_WIN.INC"
   -------------------------------------------------------------------------
   Macros to create several types of windows with the given paramenters

   Jaime Vives Piqueres, no(C)1998
   -------------------------------------------------------------------------
   Types :

     + Window_01(), classic wooden one, with 4 glasses
   -------------------------------------------------------------------------
*/

#version 3.1;

/* -------------------------------------------------------------------------
   V_Lath, creates a wood lath cutted to fit on 45§ :

   ---------------------------------------------
   \___________________________________________/

   parameters:
    -length (x)
    -thickness (z)
    -width (y)
    -wood texture
    -random seed

   returns:
    -a CSG object
   -------------------------------------------------------------------------
*/
#macro
 V_Lath(l_length,l_thick,l_width,l_wood,rn,cut45)

 #if (cut45)
  intersection{
   box{
    <-l_length*.5,-l_width*.5,-l_thick*.5>
    < l_length*.5, l_width*.5, l_thick*.5>
   }
   plane{x,0 inverse rotate 45*z translate (-l_length*.5+l_width*.5)*x}
   plane{x,0 rotate -45*z translate (l_length*.5-l_width*.5)*x}
   texture{l_wood rotate 90*y Rnd_scatrans(1,l_length,rn)}
  }
 #else
  box{
   <-l_length*.5,-l_width*.5,-l_thick*.5>
   < l_length*.5, l_width*.5, l_thick*.5>
   texture{l_wood rotate 90*y Rnd_scatrans(1,l_length,rn)}
  }
 #end

#end


/* -------------------------------------------------------------------------
   V_Glassheet, creates a sheet of glass :

   parameters:
    -heigth (y)
    -width (x)
    -thickness (z)
    -glass texture
    -random seed
    -random on/off

   returns:
    -a box of glass
   -------------------------------------------------------------------------
*/
#macro V_Glassheet(g_height,g_width,g_thick,t_glass,rn,rnd_sw)

 box{
  <-g_width*.5,-g_height*.5,-g_thick>
  < g_width*.5, g_height*.5, g_thick>
  texture{t_glass
   #if (rnd_sw)
    Rnd_scatrans(g_width*.5,g_height+g_width*2,rn)
   #end
  }
 }

#end


/* -------------------------------------------------------------------------
   Window_01, classic wooden window with 4 glasses

   parameters :
    -width
    -height
    -wood texture
    -glass texture
    -metal texture
    -left aperture angle
    -right aperture angle
    -random seed
    -glass on/off switch
    -glass randomization on/off switch

   returns:
    -a CSG object
   -------------------------------------------------------------------------
*/
#macro
 Window_01(v_width,v_height,v_thick,t_woods,t_glass,t_metal,l_aperture,r_aperture,rn,sw_glass,rnd_glass)

 #local l_w=((v_width+v_height)/2)*.05;

 union{
  // external frame
  union{
   // horizontal laths
   object{V_Lath(v_width,v_thick,l_w,t_woods,rn,on)
    translate (v_height*.5-l_w*.5)*y
   }
   object{V_Lath(v_width,v_thick,l_w,t_woods,rn,on)
    rotate 180*x
    translate -(v_height*.5-l_w*.5)*y
   }
   // vertical laths
   object{V_Lath(v_height,v_thick,l_w,t_woods,rn,on)
    rotate 90*z
    translate -(v_width*.5-l_w*.5)*x
   }
   object{V_Lath(v_height,v_thick,l_w,t_woods,rn,on)
    rotate -90*z
    translate (v_width*.5-l_w*.5)*x
   }
  }
  // internal frame
  union{
   // horizontal laths
   object{V_Lath(v_width-l_w,v_thick*.75,l_w*.5,t_woods,rn,on)
    translate (v_height*.5-l_w*1.25)*y
   }
   object{V_Lath(v_width-l_w,v_thick*.75,l_w*.5,t_woods,rn,on)
    rotate 180*x
    translate -(v_height*.5-l_w*1.25)*y
   }
   // vertical laths
   object{V_Lath(v_height-l_w,v_thick*.75,l_w*.5,t_woods,rn,on)
    rotate 90*z
    translate -(v_width*.5-l_w*1.25)*x
   }
   object{V_Lath(v_height-l_w,v_thick*.75,l_w*.5,t_woods,rn,on)
    rotate -90*z
    translate (v_width*.5-l_w*1.25)*x
   }
  }
  // left pane
  union{
   // horizontal laths
   object{V_Lath((v_width-l_w*3)*.5,v_thick*.3,l_w*.5,t_woods,rn,on)
    translate <-(v_width-l_w*3)*.25,(v_height*.5-l_w*1.75),0>
   }
   object{V_Lath((v_width-l_w*3)*.5,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate 180*x
    translate <-(v_width-l_w*3)*.25,-(v_height*.5-l_w*1.75),0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.3,l_w*.5,t_woods,rn,off)
    translate <-(v_width-l_w*3)*.25,0,0>
   }
   // vertical laths
   object{V_Lath(v_height-l_w*3,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate 90*z
    translate -(v_width*.5-l_w*1.75)*x
   }
   object{V_Lath(v_height-l_w*3,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate -90*z
    translate (-l_w*.25)*x
   }
   // upper frame pair
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    translate <-(v_width-l_w*3)*.25,(v_height*.5)-l_w*2.2,0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 180*x
    translate <-(v_width-l_w*3)*.25,l_w*.25+l_w*.2,0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 90*z
    translate <-(v_width*.5-l_w*2.2),(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate -90*z
    translate <-l_w*.7,(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   object{V_Lath((v_height-l_w*4.5-l_w*1.6)*.5,v_thick*.15,l_w*.4,t_woods,rn,off)
    rotate 90*z
    translate <-(v_width*.25-l_w*.75),(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   // lower frame pair
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    translate <-(v_width-l_w*3)*.25,-(l_w*.25+l_w*.2),0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 180*x
    translate <-(v_width-l_w*3)*.25,-((v_height*.5)-l_w*2.2),0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 90*z
    translate <-(v_width*.5-l_w*2.2),-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate -90*z
    translate <-l_w*.7,-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   object{V_Lath((v_height-l_w*4.5-l_w*1.6)*.5,v_thick*.15,l_w*.4,t_woods,rn,off)
    rotate 90*z
    translate <-(v_width*.25-l_w*.75),-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   // protector lath
   intersection{
    object{V_Lath((v_width-l_w*3)*.5,v_thick*.2,v_thick*.2,t_woods,rn,off)}
    plane{z,0 rotate -45*x
     texture{t_woods rotate 90*y Rnd_scatrans(1,v_width,rn)}
    }
    translate <-(v_width-l_w*3)*.25,-((v_height-l_w*3)*.5-v_thick*.1),l_w*.2+v_thick*.1>
   }
   // glass sheets
   #if (sw_glass)
    object{V_Glassheet((v_height-l_w*3.5),(v_width-l_w*3.5)*.5,v_thick*.05,t_glass,rn,rnd_glass)
     translate -((v_width-l_w*3)*.25)*x
    }
   #end
   // open with given aperture
   translate ((v_width-l_w*3)*.5)*x
   rotate l_aperture*y
   translate -((v_width-l_w*3)*.5)*x
  }
  // right pane
  union{
   // horizontal laths
   object{V_Lath((v_width-l_w*3)*.5,v_thick*.3,l_w*.5,t_woods,rn,on)
    translate <(v_width-l_w*3)*.25,(v_height*.5-l_w*1.75),0>
   }
   object{V_Lath((v_width-l_w*3)*.5,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate 180*x
    translate <(v_width-l_w*3)*.25,-(v_height*.5-l_w*1.75),0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.3,l_w*.5,t_woods,rn,off)
    translate <(v_width-l_w*3)*.25,0,0>
   }
   // vertical laths
   object{V_Lath(v_height-l_w*3,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate 90*z
    translate (l_w*.25)*x
   }
   object{V_Lath(v_height-l_w*3,v_thick*.3,l_w*.5,t_woods,rn,on)
    rotate -90*z
    translate (v_width*.5-l_w*1.75)*x
   }
   object{V_Lath(v_height-l_w*3,v_thick*.05,l_w*.2,t_woods,rn,off)
    rotate 90*z
    translate -v_thick*.155*z
   }
   // upper frame pair
   union{
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    translate <-(v_width-l_w*3)*.25,(v_height*.5)-l_w*2.2,0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 180*x
    translate <-(v_width-l_w*3)*.25,l_w*.25+l_w*.2,0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 90*z
    translate <-(v_width*.5-l_w*2.2),(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate -90*z
    translate <-l_w*.7,(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   object{V_Lath((v_height-l_w*4.5-l_w*1.6)*.5,v_thick*.15,l_w*.4,t_woods,rn,off)
    rotate 90*z
    translate <-(v_width*.25-l_w*.75),(v_height-l_w*4.5)*.25+l_w*.25,0>
   }
   // lower frame pair
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    translate <-(v_width-l_w*3)*.25,-(l_w*.25+l_w*.2),0>
   }
   object{V_Lath((v_width-l_w*5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 180*x
    translate <-(v_width-l_w*3)*.25,-((v_height*.5)-l_w*2.2),0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate 90*z
    translate <-(v_width*.5-l_w*2.2),-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   object{V_Lath((v_height-l_w*4.5)*.5,v_thick*.15,l_w*.4,t_woods,rn,on)
    rotate -90*z
    translate <-l_w*.7,-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   object{V_Lath((v_height-l_w*4.5-l_w*1.6)*.5,v_thick*.15,l_w*.4,t_woods,rn,off)
    rotate 90*z
    translate <-(v_width*.25-l_w*.75),-((v_height-l_w*4.5)*.25+l_w*.25),0>
   }
   translate (v_width*.5-l_w*1.5)*x
   }
   // protector lath
   intersection{
    object{V_Lath((v_width-l_w*3)*.5,v_thick*.2,v_thick*.2,t_woods,rn,off)}
    plane{z,0 rotate -45*x
     texture{t_woods rotate 90*y Rnd_scatrans(1,v_width,rn)}
    }
    translate <(v_width-l_w*3)*.25,-((v_height-l_w*3)*.5-v_thick*.1),l_w*.2+v_thick*.1>
   }
   // glass sheets
   #if (sw_glass)
    object{V_Glassheet((v_height-l_w*3.5),(v_width-l_w*3.5)*.5,v_thick*.05,t_glass,rn,rnd_glass)
     translate ((v_width-l_w*3)*.25)*x
    }
   #end
   translate -((v_width-l_w*3)*.5)*x
   rotate -r_aperture*y
   translate ((v_width-l_w*3)*.5)*x
  }
 }

#end
