//==========================================
// The fortress of Carcassonne
// -----------------------------------------
// Made for Persistence of vision 3.6
//==========================================  
// Copyright 2001-2004 Gilles Tran http://www.oyonale.com
// -----------------------------------------
// This work is licensed under the Creative Commons Attribution License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/2.0/ 
// or send a letter to Creative Commons, 559 Nathan Abbott Way, Stanford, California 94305, USA.
// You are free:
// - to copy, distribute, display, and perform the work
// - to make derivative works
// - to make commercial use of the work
// Under the following conditions:
// - Attribution. You must give the original author credit.
// - For any reuse or distribution, you must make clear to others the license terms of this work.
// - Any of these conditions can be waived if you get permission from the copyright holder.
// Your fair use and other rights are in no way affected by the above. 
//==========================================  // Render this image in portrait format
// This picture requires radiosity 

// Cmd:+w240 +h320 +a


// uncomment the line below when no radiosity used
//light_source{<-23, 13, -120.0> color rgb<84,129,199>/255} 


#include "colors.inc"
#include "functions.inc"
#declare HQOn=0; // low quality radiosity
#declare HQOn=1; // high quality radiosity

global_settings{
        assumed_gamma 1 // to be changed if too pale
        max_trace_level 15
        radiosity{
            #switch (HQOn)
                #case (0)
                    count 50 error_bound 2 
                #break
                #case (1)
                    count 200 error_bound 0.05 
                #break
            #end    
            nearest_count 4
            recursion_limit 1
            low_error_factor 1
            gray_threshold 0  
            minimum_reuse 0.015
            brightness 3       
            adc_bailout 0.01/2 
            normal on
        }
}        

//=========================================
// macro mAlign
//-----------------------------------------
// returns a matrix operation that aligns an object or texture along P2-P1
// the object is translated to P1
// translate to P2-P1 if you want the object to be on P2
#macro mAlign(P1,P2)
#local yV1=vnormalize(P2-P1);
#local xV1=vnormalize(vcross(yV1,z));
#local zV1=vcross(xV1,yV1);
    matrix <xV1.x,xV1.y,xV1.z,yV1.x,yV1.y,yV1.z,zV1.x,zV1.y,zV1.z,P1.x,P1.y,P1.z>
#end   


//---------------------------
// Various common parameters
//---------------------------
#declare C_Stone0=rgb 0.6*<98,80,50>/255;
#declare C_Stone1=rgb 0.2*<206,184,158>/255;
#declare C_Stone2=rgb 0.2*<227,208,184>/255;
#declare C_Moss=rgb 0.2*<128,127,144>/255;
#declare C_Roof=<0.355,0.354,0.302>;

#declare T_Ground=texture{
    pigment{
        slope y
        color_map{
            [0.9 color C_Stone2*0.5]
            [0.95 color C_Stone2*0.1]
            [1 color ForestGreen*0.1]
        }
    }
    finish{ambient 0 diffuse 1}
}                                       
#declare T_Tower=texture{
    pigment{
        color C_Stone1
    }
    finish{ambient 0 diffuse 1 specular 0.2 roughness 0.1}
}                                       
#declare xCren=3.71; // Crenel length
#declare xCrenLH=0.61; // Large loophole width
#declare xCren2=2.45; // End crenel length
#declare xCren3=xCren-xCrenLH; // Top crenel length
#declare yCren=2.2; // Crenel height
#declare yCrenLH=1.22; // Large loophole height
#declare zCren=0.75; // Crenel width 


#declare zW=2.25; // Wall width
#declare zR=6.1; // Road width = building 1 width

//---------------------------
// Macro that folds a height field around a corner
//---------------------------
#macro mHFCorner(HF,xHF,yHF,zHF,rCorner,nCorner)
    // the HF must be already textured and vertical
    // it must have been translated before to be in the positive z
    // xHF = width of front pane (x, 0 -> xHF)
    // yHF = height of side pane (y, 0 -> yHF)
    // zHF = depth of side pane (z, 0 -> zHF)
    // rCorner = radius of corner
    // nCorner = number of corner elements
    
    #local lHF=xHF+zHF+rCorner*(-2+pi/2);
    #local HF=object{HF scale <lHF,yHF,1>}
    #local iCorner=(rCorner*pi/2)/nCorner;
    #local i=0;
    union{
        object{HF clipped_by{plane{x,xHF-rCorner}}}
        #while (i<nCorner)
            object{HF clipped_by{box{<0,-0.1,-1.1>,<iCorner,yHF*1.1,1.1> translate x*(xHF-rCorner+i*iCorner)}}
                translate -x*(xHF-rCorner+(i+0.5)*iCorner)
                translate z*-rCorner 
                rotate -y*(i*90/nCorner+45/nCorner) 
                translate z*rCorner+x*(xHF-rCorner)
            }
            #local i=i+1;
        #end    
        object{HF clipped_by{plane{x,lHF-zHF+rCorner inverse}} translate -x*(lHF-rCorner-zHF) rotate y*-90 translate <xHF,0,-rCorner>} // pigment{Cyan}}
    }
#end

//---------------------------
// Wall 1
//---------------------------
#declare yW0=100;
#declare xW1=8*xCren+xCren3;
#declare yW1=9.93;
#declare yW1a=yW1-yCrenLH;
#declare yW1b1=6.02;
#declare yW1b2=5.3;
#declare yW1b3=4.8;
#declare yW1b4=4.3;
#declare yW1b5=4;
#declare xW1b1=7;
#declare xW1b2=2.5;
#declare xW1b3=2.8;
#declare xW1b4=2.3;
#declare xW1b5=2.6;
#declare zW1b=1.2; 
#declare xW1b=xW1b1+xW1b2+xW1b3+xW1b4+xW1b5;
#declare T_Wall1bLayer=texture{
        pigment{
            function {min(1,max(0,y))}
            turbulence 0.5 lambda 3 poly_wave 2
            color_map{                     
                [0 rgbf <98/255,80/255,50/255,1>]
                [0.5 rgbf <98/255,80/255,50/255,3>*0.1]
                [1 rgb <98/255,80/255,50/255>*0.4]
            }
        }
        scale yW1b1
}
#declare C_Wall1=color rgb <35,29,19>/255;
#declare C_Wall2=color rgb <54,44,28>/255;
#declare T_Wall=texture{
    pigment{
        crackle
        turbulence 1
        color_map{[0 C_Wall1][1 C_Wall2]}
    }
    finish{ambient 0 diffuse 1}        
}
#declare hfWall1=height_field{jpeg "hfwall1" water_level 0.001}
#declare hfWall1a=height_field{jpeg "hfwall1a" water_level 0.001}
#declare hfWall1b=height_field{jpeg "hfwall1b" water_level 0.001}
#declare hfWall1b1=height_field{jpeg "hfwall1b1" water_level 0.001 texture{T_Wall} texture{T_Wall1bLayer rotate x*90}}
#declare hfWall1b2=height_field{jpeg "hfwall1b2" water_level 0.001 texture{T_Wall} texture{T_Wall1bLayer rotate x*90}}
#declare hfWall1b3=height_field{jpeg "hfwall1b3" water_level 0.001 texture{T_Wall} texture{T_Wall1bLayer rotate x*90}}
#declare hfWall1b4=height_field{jpeg "hfwall1b4" water_level 0.001 texture{T_Wall} texture{T_Wall1bLayer rotate x*90}}
#declare hfWall1b5=height_field{jpeg "hfwall1b5" water_level 0.001 texture{T_Wall} texture{T_Wall1bLayer rotate x*90}}
    
#declare Wall1_0=union{
//    box{-yW0*y,<xW1,yW1a,zW>}
    object{hfWall1 rotate -x*90 scale <xW1,-yW1,0.1>}
    object{hfWall1 rotate -x*90 scale <xW1,-yW1*1.3,0.1> translate -y*yW1}
    object{hfWall1 rotate -x*90 scale <xW1,yW1,0.1>}
    object{hfWall1a rotate -x*90 scale <zW,yW1,0.1> 
        texture{T_Wall}
        texture{
            pigment{
                function {min(1,max(0,y))}
                turbulence 0.2 lambda 3
                poly_wave 2
                color_map{
                    [0 rgbf <98/255,80/255,50/255,1>]
                    [(yW1-yCren)/yW1 rgbf <98/255,80/255,50/255,3>*0.1]
                    [1 rgb <98/255,80/255,50/255>*0.4]
                }
            }
            scale yW1*<0.3,1,0.3>
        }

        rotate y*-90 translate x*xW1
    }
    union{ // Crenels
        #declare i=0;
        #while (i<9)
            #if (i<8)
                box{0,<xCren3,yCrenLH,zCren> translate x*i*xCren}
            #else
                box{0,<xCren3,yCrenLH,zW> translate x*i*xCren}
            #end                
            #declare i=i+1;
        #end
        translate y*yW1a
    }          

}    
#declare Wall1_1=object{Wall1_0
        texture{T_Wall} 
        texture{
            pigment{
                function {min(1,max(0,y))}
                turbulence 0.2 lambda 3
                poly_wave 2
                color_map{
                    [0 rgbf <98/255,80/255,50/255,1>]
                    [(yW1-yCren)/yW1 rgbf <98/255,80/255,50/255,3>*0.1]
                    [1 rgb <98/255,80/255,50/255>*0.4]
                }
            }
            scale yW1*<0.1,1.3,1>
            translate -y*0.3*yW1
            translate x*50
        }

}
#declare Wall1_2=object{Wall1_0
        texture{T_Wall} 
        texture{
            pigment{
                function {min(1,max(0,y))}
                turbulence 0.3 lambda 4
                poly_wave 2
                color_map{
                    [0 rgbf <98/255,80/255,50/255,1>]
                    [0.5 rgbf <98/255,80/255,50/255,3>*0.1]
                    [1 rgb <98/255,80/255,50/255>*0.4]
                }
            }
            scale yW1*<0.1,2,1>
            translate y*-yW1
            //translate -y*50
        }

}

#declare Wall1=union{
    object{Wall1_0
        texture{T_Wall} 
        texture{
            pigment{
                function {min(1,max(0,y))}
                turbulence 0.2 lambda 3
                poly_wave 2
                color_map{
                    [0 rgbf <98/255,80/255,50/255,1>]
                    [(yW1-yCren)/yW1 rgbf <98/255,80/255,50/255,3>*0.1]
                    [1 rgb <98/255,80/255,50/255>*0.4]
                }
            }
            scale yW1*<0.1,1,1>
        }

    }
    // small wall 1
    union{
        box{-yW0*y,<xW1b1,yW1b1,zW1b>}
        box{-yW0*y,<xW1b2,yW1b2,zW1b> translate x*xW1b1}
        box{-yW0*y,<xW1b3,yW1b3,zW1b> translate x*(xW1b1+xW1b2)}
        box{-yW0*y,<xW1b4,yW1b4,zW1b> translate x*(xW1b1+xW1b2+xW1b3)}
        box{-yW0*y,<xW1b5,yW1b5,zW1b> translate x*(xW1b1+xW1b2+xW1b3+xW1b4)}
        object{hfWall1b rotate -x*90 scale <xW1b,yW1b1,0.05>}
        object{hfWall1b1 rotate -x*90 scale <zW1b,(yW1b1-yW1b2),0.05> rotate y*-90 translate x*(xW1b1)+y*yW1b2}
        object{hfWall1b2 rotate -x*90 scale <zW1b,(yW1b2-yW1b3),0.05> rotate y*-90 translate x*(xW1b1+xW1b2)+y*yW1b3}
        object{hfWall1b3 rotate -x*90 scale <zW1b,(yW1b3-yW1b4),0.05> rotate y*-90 translate x*(xW1b-xW1b4-xW1b5)+y*yW1b4}
        object{hfWall1b4 rotate -x*90 scale <zW1b,(yW1b4-yW1b5),0.05> rotate y*-90 translate x*(xW1b-xW1b5)+y*yW1b5}
        object{hfWall1b5 rotate -x*90 scale <zW1b,yW1b5,0.05> rotate y*-90 translate x*xW1b}

        #declare e=0.01;
        #declare n=0.01;                                  
        #declare TopWallb=
        isosurface{
//         	function {(( abs(x)^(2/e)+abs((2*y-1))^(2/e))^(e/n)+abs(z)^(2/n)-1)*(1-y^3) + (x*x+z*z+y*y)*y^3}
//         	function {(( Q+A^B)^C+D^E-1)*(1-y^3) + (x*x+z*z+y*y)*y^3}
         	function {(pow(( pow(abs(x),2/e)+pow(abs(2*y-1),2/e)),e/n)+pow(abs(z),2/n)-1)*(1-pow(y,3)) + (x*x+z*z+y*y)*pow(y,3)}
        	contained_by { box { <-1,0,-1>, <1,1,1> } }
        	threshold 0                   
            max_gradient 10
        }

        union{
            object{TopWallb scale <xW1b1*2,0.3,zW1b>*0.5 translate <0,yW1b1,zW1b/2>}
            object{TopWallb scale <xW1b2*2,0.3,zW1b>*0.5 translate <xW1b1,yW1b2,zW1b/2>}
            object{TopWallb scale <xW1b3*2,0.3,zW1b>*0.5 translate <xW1b1+xW1b2,yW1b3,zW1b/2>}
            object{TopWallb scale <xW1b4*2,0.3,zW1b>*0.5 translate <xW1b1+xW1b2+xW1b3,yW1b4,zW1b/2>}
            object{TopWallb scale <xW1b5*2,0.3,zW1b>*0.5 translate <xW1b1+xW1b2+xW1b3+xW1b4,yW1b5,zW1b/2>}
            texture{T_Wall}
            translate -z*0.02
        }
        texture{T_Wall}
        texture{T_Wall1bLayer}
        translate <xW1,0,zW-zW1b>
    }
    // small wall 2
    union{
        box{-yW0*y-x*xW1b1,<xW1b1,yW1b1,zW1b>}
        box{-yW0*y,<xW1b2,yW1b2,zW1b> translate x*xW1b1}
        box{-yW0*y,<xW1b3,yW1b3,zW1b> translate x*(xW1b1+xW1b2)}
        box{-yW0*y,<xW1b4,yW1b4,zW1b> translate x*(xW1b1+xW1b2+xW1b3)}
        box{-yW0*y,<xW1b5,yW1b5,zW1b> translate x*(xW1b1+xW1b2+xW1b3+xW1b4)}
        object{hfWall1b rotate -x*90 scale <xW1b,yW1b1,0.1>}
        object{hfWall1b rotate -x*90 scale <xW1b,-yW1b1,0.1>}


        union{
            object{TopWallb scale <xW1b1*2,0.3,zW1b>*0.51 translate <0,yW1b1,zW1b/2>}
            object{TopWallb scale <xW1b2*2,0.3,zW1b>*0.51 translate <xW1b1,yW1b2,zW1b/2>}
            object{TopWallb scale <xW1b3*2,0.3,zW1b>*0.51 translate <xW1b1+xW1b2,yW1b3,zW1b/2>}
            object{TopWallb scale <xW1b4*2,0.3,zW1b>*0.51 translate <xW1b1+xW1b2+xW1b3,yW1b4,zW1b/2>}
            object{TopWallb scale <xW1b5*2,0.3,zW1b>*0.51 translate <xW1b1+xW1b2+xW1b3+xW1b4,yW1b5,zW1b/2>}
            texture{T_Wall}
            translate -z*0.02
        }

        texture{T_Wall}
        texture{T_Wall1bLayer scale <0.1,1,1> translate x*20}
        scale <-1,1,1>
        translate <xW1b,0,zW-zW1b>
        rotate y*10
        translate <2.5+xW1+xW1b,0,0>
    }        
    #declare e=0.1;         
    #declare n=0.1;         
    #declare yStep=0.224;
    #declare FPig1=function{pigment{brick color White, color Black turbulence 0.5 scale 0.2}}
    #declare FPig2=function{pigment{crackle turbulence 0.1 translate 1}}
    #declare WStep=isosurface {
//    	function {( abs(x)^(2/e)+abs(y)^(2/e))^(e/n)+abs(z)^(2/n)-1 + 0.2*FPig1(x*0.8,y*0.8,z*0.8).gray+ 0.8*FPig2(x*0.8,y*0.8,z*0.8).gray}
    	function { pow((pow(abs(x),2/e)+pow(abs(y),2/e)),e/n)+pow(abs(z),2/n)-1 + 0.2*FPig1(x*0.8,y*0.8,z*0.8).gray+ 0.8*FPig2(x*0.8,y*0.8,z*0.8).gray}
    	contained_by { box { -1, 1 } }
    	threshold 0                   
    	max_gradient 14
    }               
    union{ // steps
        object{WStep scale <2,yStep,2.4>}            
        object{WStep scale <1.75,yStep*2,2>}            
        object{WStep scale <1.5,yStep*3,1.5>}            
        scale <0.8,1,1>
        rotate -z*7
        translate <(xW1+xW1b+1.5),-yStep,2>
        texture{T_Wall} 
    }
    texture{T_Wall} 
    texture{
        pigment{
            gradient y
            turbulence 1 lambda 3
            poly_wave 2
            color_map{
                [0 rgbf <98/255,80/255,50/255,1>]
                [(yW1-yCren)/yW1 rgbf <98/255,80/255,50/255,3>*0.1]
                [1 rgb <98/255,80/255,50/255>*0.4]
            }
        }
        scale yW1*<0.1,1,1>
    }
}                           

//---------------------------
// Wall 2
//---------------------------
#declare xW2=xW1;
#declare yW2=16.8;
#declare yW2a=yW2-yCrenLH;
#declare zW2=zW;
#declare Wall2=union{
    box{0,<xW2,yW2a,zW>}
    object{hfWall1 scale <-1,1,1> translate x rotate -x*90 scale <xW1,yW1,0.1> translate y*(yW2-yW1)}
    union{ // Crenel
        #declare i=0;
        #while (i<9)
            box{0,<xCren3,yCrenLH,zCren> translate x*i*xCren}
            #declare i=i+1;
        #end
        translate y*yW2a
    }
    texture{T_Wall} 
    texture{
        pigment{
            function {min(1,max(0,y))}
            turbulence 1 lambda 3
            poly_wave 2
            color_map{
                [0 rgbf <98/255,80/255,50/255,1>]
                [(yW2-yCren)/yW2 rgbf <98/255,80/255,50/255,3>*0.1]
                [1 rgb <98/255,80/255,50/255>*0.4]
            }
        }
        scale yW2*<0.1,1,0.1>
    }
}             
#declare T_Windows=
        texture{
            gradient y
            texture_map{
                [0.95 pigment{Black} normal{bozo 0.5} finish{reflection 0.2}]
                [0.95 pigment{bozo turbulence 0.5 scale 0.5 color_map{[0 C_Stone0][1 C_Stone0*0.5]}}]
            }
            scale 0.8                
        }


//---------------------------
//       Title: Brick macro
//      Author: Jeff Lee <shipbrk@gate.net>
//  Adaptation: Gilles Tran
// Description: Creates a decent brick pattern, with random variation in brick colour.
//     Version: 1.0
//        Date: 19 February 1998
//    Platform: POV-Ray 3.0
//---------------------------

#macro pigBrick(BrickWidth,BrickHeight,BrickDepth,MortarWidth,BrickColor,MortarColor,BrickScale)
  #local P_BrickA = pigment { 
    bozo
    pigment_map {
      [0.00 BrickColor*0.7]
      [0.2 BrickColor*0.5]
      [0.8 BrickColor]
      [1.00 BrickColor*0.7]
    }
    scale BrickScale
  }
  
  #local P_BrickB = pigment {P_BrickA    warp { repeat x*BrickWidth offset <0,15*BrickHeight,0> }  }
  #local P_BrickC = pigment {P_BrickB    warp { repeat z*BrickDepth offset <0,20*BrickHeight,0> }  }
  #local P_BrickD = pigment {P_BrickC    warp { repeat y*BrickHeight offset <BrickWidth*7.5,0,BrickDepth*6.5> }    translate z*(BrickDepth/2)  }
  #local P_BrickMortar = pigment {MortarColor }
  #declare P_Brick = pigment {brick pigment { P_BrickMortar }, pigment { P_BrickD }    brick_size <BrickWidth,BrickHeight,BrickDepth>    mortar MortarWidth  }
  
#end
//---------------------------
// End of brick macro
//---------------------------
// Building 1
//---------------------------
#declare xB1=15.2;
#declare yB1=20.2;
#declare zB1=zR;
#declare yRFB1=4.2;
#declare zRFB1=2;
#declare eRFB1=0.3;
#declare T_RFB1=texture{
    #declare BrickWidth=0.4;
    #declare BrickHeight=0.3;
    #declare BrickDepth=0.3;
    #declare MortarWidth=0.01;
    #declare BrickColor=rgb C_Roof*0.5;
    #declare MortarColor=BrickColor*0.1;
    #declare BrickScale=1;
    
    pigBrick(BrickWidth,BrickHeight,BrickDepth,MortarWidth,BrickColor,MortarColor,BrickScale)
    pigment{P_Brick}
    finish{ambient 0 diffuse 1}

}
#declare Build1=union{
    union{
        box{y*yB1*0.9,<xB1,yB1,zB1> translate <-0.3,0,0.3>}
        box{0,<xB1,yB1,0.2> translate <-0.3,0,0.3>}
        texture{T_Windows}
    }
    #declare hfBuild1_0=height_field{jpeg "hfbuild1" water_level 0.01
        texture{T_Wall}
        translate -y scale <1,0.05,1>
        rotate -x*90
    }
    #declare hfBuild1=object{mHFCorner(hfBuild1_0,xB1,yB1,zB1,0.2,5)}
    object{hfBuild1}
    object{hfBuild1 scale <1,-1,1> rotate x*5}
    
    height_field{
        jpeg "hfdoor" // inner door
        texture{T_Wall}
        rotate x*-90
        scale <zB1*0.8,yB1*0.34,0.1>
        translate x*zB1*0.2
        rotate y*-90
        translate <xB1*0.5,yB1*0.5,0>            
    }
    // Roof
    union{
        #declare P1=<-eRFB1,0,-eRFB1>;
        #declare P2=<xB1+eRFB1,0,-eRFB1>;
        #declare P3=<xB1*0.5,yRFB1,zRFB1>;
        #declare P4=<xB1+eRFB1,0,zB1>;
        #declare P5=<-eRFB1,0,zB1>;
        #declare P6=<xB1*0.5,yRFB1,zB1>;
        triangle{P1,P2,P3 texture{T_RFB1 rotate x*degrees(atan2(zB1-zRFB1,yRFB1))}}
        polygon{5,P2,P3,P6,P4,P2 texture{T_RFB1 rotate y*90 rotate z*degrees(atan2(xB1/2,yRFB1))}}
        polygon{5,P1,P3,P6,P5,P1 texture{T_RFB1 rotate y*90 rotate -z*degrees(atan2(xB1/2,yRFB1))}}
        union{
            lathe {
                cubic_spline 7,<0,0>,<0.8,0>,<0.8,2.85>,<2.2,3.01>,<1,3.5>,<0.01,7>,<0.01,0>
                scale 0.08*<1,1.5,1>
                translate z*0.05
            }

            #declare i=0;
            #declare n=10;
            #while (i<n)
                cone{0,0.1,z*(zB1-zRFB1)/n,0.15 translate z*i*(zB1-zRFB1)/n}
                #declare i=i+1;
            #end
            texture{
                pigment{
                    bozo
                    color_map{
                        [0 BrickColor]
                        [1 BrickColor*0.6]
                    }
                }
                finish{ambient 0 diffuse 1}
            }
            translate <xB1/2,yRFB1,zRFB1>
        }
        translate y*yB1
    }
    pigment{Red}
}
//---------------------------
// Building 2
//---------------------------
#declare xB2=xB1-1; // 15.2
#declare yB2a=25.13;
#declare yB2=yB2a-yW2; // 8.33
#declare zB2=xB1;  
#macro mTiles(xTiles,zTiles)
union{
    #local xTile=0.32;    
    #local zTile=0.42;    
    #local Tile=difference{
            cylinder{0,zTile*z*1.3,xTile*0.5}
            cylinder{-z*0.1,zTile*z*1.1,xTile*0.5*0.8}
            plane{y,0}
            scale <1,0.8,1>
            translate x*xTile*0.5
    }                                         
    
    #local j=0;
    #local rd=seed(8989);
    #local nxTiles=int(xTiles/xTile);
    #local nzTiles=int(zTiles/zTile);
    #while (j<nzTiles)
        #local i=j*2;
        #while (i<(nxTiles-j*2))
            object{
                Tile 
                rotate x*5*(1+rand(rd))
                translate x*i*xTile
                translate z*j*zTile
            }
            #local i=i+1;
        #end            
        #local j=j+1;
    #end
    scale <xTiles/(nxTiles*xTile),1,zTiles/(nzTiles*zTile)>
    rotate x*-30
}
#end            

#declare C_TileB2 = rgb 0.15*<0.955,0.603,0.235>;
#declare T_TilesB2=texture{
            pigment{bozo turbulence 1 lambda 3 color_map{[0.2 C_TileB2][0.8 C_TileB2*0.5]}}
            finish{ambient 0 diffuse 1}
        }
#declare Build2=union{
    box{0,<xB2,yB2,zB2> translate <-0.3,0,0.3> texture{T_Windows}}                    
    #declare hfBuild2_0=height_field{jpeg "hfbuild2" water_level 0.01
        texture{T_Wall}
        translate -y scale <1,0.1,1>
        rotate -x*90
    }
    #declare hfBuild2=
        union{
            object{mHFCorner(hfBuild2_0,xB2,yB2,zB2,0.2,5)}
            union{
                object{mTiles(xB2,1) translate <0,0,-0.2>}
                object{mTiles(zB2,1) rotate y*-90 translate <xB2+0.2,0,-0.2>}
                translate y*(yB2-0.15)
                texture{T_TilesB2}
            }
        }
    object{hfBuild2}
    object{hfBuild2 scale <1,-1,1>}
    translate y*yW2+x*1.1
}
//---------------------------
// Building 3
//---------------------------

#declare xB3=xCren*6+xCren3;
#declare yB3=yW1*0.9;
#declare yB3a=yB3-yCrenLH;
#declare zB3=37.3;
#declare hfBuild3=height_field{
    jpeg "hfbuild3" 
    water_level 0.01
    texture{T_Wall}
    rotate x*-90
    scale <1,1,0.2>
}
#declare xB3a=3;
#declare yB3a=247*xB3a/161;
#declare Build3=object{
    mHFCorner(hfBuild3,xB3a,yB3a,xB3a,0.2,5)
    rotate y*90
    translate z*xB3a
}
//---------------------------
// Building 4
//---------------------------
#declare xB4=xCren*8+xCren3;
#declare yB4=yB3*0.8;
#declare yB4a=yB4-yCrenLH;
#declare zB4=37.3;
#declare Build4=union{
    box{-y*yW0,<xB4,yB4a,zB4>}
    union{ // Crenels
        #declare i=0;
        #while (i<8)
            #if (i<7)
                box{0,<xCren3,yCrenLH,zCren> translate x*i*xCren}
            #else
                box{0,<xCren3,yCrenLH,zB4> translate x*i*xCren}
            #end                
            #declare i=i+1;
        #end
        translate y*yB4a
    }               
}    
//---------------------------
// Square tower
//---------------------------
#declare xST=11.2;
#declare yST=51.4;
#declare xCrenST=xST/6.4;
#declare xCrenSTLH=xCrenST*0.8;
#declare yCrenST=xCrenST*1.2;
#declare hfSTower_0=height_field{
    jpeg "hfstower" 
    water_level 0.1
    texture{T_Wall}
    translate -y*0.8
    scale <1,0.1,1>
    rotate x*-90
}
#declare hfSTower=object{mHFCorner(hfSTower_0,xST,yST+yCrenST,xST,0.2,5)}
#declare STower=union{
    box{<0.15,0,0.15>,<xST-0.15,yST,xST-0.15> pigment{Black}}
    object{hfSTower}
    union{
        #declare i=0;
        #while (i<4)
            box{<0,0,zCren*0.1>,<xCrenST-0.1,yCrenST,zCren*1.5> translate x*((xCrenST+xCrenSTLH)*i)}
            box{<0,0,zCren*0.1>,<xCrenST-0.1,yCrenST,zCren*1.5> translate x*((xCrenST+xCrenSTLH)*i) scale <1,1,-1> translate z*xST}
            box{<0,0,zCren*0.1>,<xCrenST,yCrenST,zCren*1.5> translate x*((xCrenST+xCrenSTLH)*i) rotate y*-90 translate x*xST}
            #declare i=i+1;
        #end        
        translate y*yST
    }
}    
//---------------------------
// Tower
//---------------------------
#declare rd=seed(0);
#declare rT=4.19; // tower radius
#declare yT=25.54; // tower height
#declare yTR=9.6; // tower roof height
#declare rTRS=0.2; // tower spire base radius
#declare yTRS=3.43; // tower spire height
#declare eTR=0.4; // roof ledge width
#declare rTR=rT+eTR; // tower roof radius
#declare xT2=2.53; // secondary tower width
#declare xTR2=xT2+eTR; // secondary tower width
#declare yTR2=4; // secondary tower roof height
#declare yTRS2=1.5; // secondary tower spire height
#declare T_Beam=texture{
    pigment{
        bozo turbulence 1 lambda 3
        color_map{
            [0.5 C_Stone0]
            [0.55 C_Stone0*0.4]
        }
    }                  
    finish{ambient 0 diffuse 1}
}
#declare C_Spire=rgb <0.385,0.364,0.302>;
#declare T_Spire=texture{
    pigment{
        gradient y
        pigment_map{
            [0 crackle solid scale 0.02 color_map{[0 C_Spire*0.1][1 C_Spire*1]}]
            [1 crackle solid scale 0.02 color_map{[0 C_Spire*0.5][1 C_Spire*1.5]}]
        }            

    }
    finish{ambient 0 diffuse 1 metallic brilliance 4 specular 1 roughness 0.01} //  reflection 0.2}
}

#declare T_WMesh=texture{
    pigment{C_Stone0*0.3}
    finish{ambient 0 diffuse 1 specular 0.001 roughness 0.1}
}            

#macro mWindowMesh(xW,yW)
    #local xt=10;
    #local yt=xt*yW/xW;
    union{
        isosurface{
            function{f_mesh1(x,y,z,3, 3, 0.8, 0.5,  1)}
            contained_by{ box {<-xt/2, -0.5, -yt/2>, <xt/2, 0.5, yt/2>}}
            max_gradient 1.3
            threshold 0.18
            rotate -x*90
            scale <xW/xt,yW/yt,xW/xt>
        }
        box{<-xW/2,-yW/2,0.1>, <xW/2, yW/2, 1> texture{pigment{Black} finish{ambient 0 diffuse 0 reflection 0.3}}        }
    }
#end
#macro mWindowMesh2(xW,yW)
    #local xt=10;
    #local yt=xt*yW/xW;
    isosurface{
        function{f_mesh1(x,y,z,1.5, 1.5, 0.8, 0.25,  1)+f_noise3d(x*10,y*10,z*10)*0.1}
        contained_by{ box {<-xt/2, -0.5, -yt/2>, <xt/2, 0.5, yt/2>}}
        max_gradient 1.3
        threshold 0.20
        rotate -x*90
        scale <xW/xt,yW/yt,xW/xt>
    }
#end

#declare Grid=object{mWindowMesh2(2,3) texture{pigment{C_Stone0*0.3} finish{ambient 0 diffuse 1}}}
#declare hfBrick=function{pigment{image_map{jpeg "hftower" map_type 2} turbulence 0.01}}
#declare T_Tower=texture{T_Wall}
#declare T_Tower2=texture{
    pigment{
        pigment_pattern{
            gradient y
            turbulence 1
            color_map{[0 White][1 Black]}
            scale 0.2
        }                
        pigment_map{
            [0
        
                slope y
                pigment_map{
                    [0 bozo turbulence 1 lambda 3 color_map{[0.3 C_Stone0][0.6 C_Stone0*2] }scale 0.025]
                    [0.5 bozo turbulence 1 lambda 3 color_map{[0.3 C_Stone0][0.6 C_Stone0*2] }scale 0.05]
                    [0.6 color C_Moss*0.3]
                }
            ]                    
            [1
        
                slope y
                pigment_map{
                    [0 bozo turbulence 1 lambda 3 color_map{[0.3 C_Stone1][0.6 C_Stone2*2] }scale 0.025]
                    [0.4 bozo turbulence 1 lambda 3 color_map{[0.3 C_Stone2][0.6 C_Stone0*2] }scale 0.05]
                    [0.5 color C_Moss*0.3]
                }
            ]
        }                                        
    }
    normal{bumps 1 turbulence 1 scale 0.01} 
    finish{ambient 0 diffuse 1}
}
// main tower
#declare rs=0.03; // try several rs values <1; the higher it is the higher is the text
#declare TowerBody=union{
    difference{
        
        isosurface{
            function{x*x+z*z + rs - hfBrick(x,y,z).gray*rs}
            contained_by{box{-1,1}}
            max_gradient 10 
            threshold 1
        }
        
        //cylinder{0,y,1}      
        // window holes            
        superellipsoid{<0.15,0.12> scale <0.3/rT,0.9/yT,1> translate -z+y*0.92 rotate -y*55}
        superellipsoid{<0.1,0.1> scale <0.31/rT,0.87/yT,1> translate -z+y*0.918 rotate -y*25}
        superellipsoid{<0.1,0.1> scale <0.31/rT,0.87/yT,1> translate -z+y*0.9 rotate y*16}
        union{
            superellipsoid{<0.2,0.2>}
            sphere{y,1}
            scale <0.35/rT,0.87/yT,1> 
            translate -z+y*0.52 rotate -y*5
        }
        superellipsoid{<0.2,0.2> scale <0.25/rT,0.53/yT,1> translate -z+y*0.33 rotate -y*11}
        superellipsoid{<0.4,0.4> scale <0.1/rT,0.5/yT,1> translate -z+y*0.72 rotate -y*20}
        texture{T_Tower}
        scale <rT,yT,rT>
    }        
    union{// windows
        object{mWindowMesh(0.6,1.8) translate -z*(rT-0.1)+y*0.92*yT rotate -y*55}
        object{mWindowMesh(0.62,1.9) translate -z*(rT-0.1)+y*0.918*yT rotate -y*25}
        object{mWindowMesh(0.62,1.9) translate -z*(rT-0.1)+y*0.9*yT rotate y*16}
        object{mWindowMesh(0.8,3.2) translate -z*(rT-0.1)+y*0.52*yT rotate -y*5}
        texture{T_WMesh}
    }            
}
// close up tower on the right
#declare rs=0.03; // try several rs values <1; the higher it is the higher is the text
#declare TowerBody2=union{
    isosurface{
        function{x*x+z*z + rs - hfBrick(x,y,z).gray*rs + f_noise3d(x*10,y*10,z*10)*0.25}
        contained_by{box{-1,1}}
        max_gradient 10
        threshold 1
        texture{T_Tower2}
        scale <rT,yT,rT>
    }            
    object{Grid translate <0,15.5,-rT-0.01> rotate y*5}
}

#declare Tower=union{  
    object{TowerBody}    
    union{ // roof
        #declare i=0;
        #declare n=50;      
        #declare xRFT=0.2;
        #declare yRFT=0.02;
        #declare RoofTile=box{<-yRFT,-yRFT,-xRFT/2>,<1,yRFT,xRFT/2>}
        #declare iStep=0.2;
        #declare nPow=1.5;
        #declare rTR1=(rTR-rTRS)*pow(1-iStep,nPow)+rTRS;
        union{
            #while (i<1)
                #if (i<iStep) // bottom part (curved)
                    #declare Pos=<-(rTR-rTRS)*pow(1-i,nPow)-rTRS,yTR*i,0>;
                #else // top part (straight)
                    #declare Pos=<(rTR1-rTRS)*i/(1-iStep)+((iStep*rTRS-rTR1)/(1-iStep)),yTR*i,0>;
                #end
                #declare j=0;
                #ifdef(PosOld)         
                    #declare teta=degrees(atan2(Pos.y-PosOld.y,Pos.x-PosOld.x))-5;
                    #declare sc=vlength(Pos-PosOld);
                    #while (j<2*pi*abs(PosOld.x))
                        object{RoofTile scale <sc*1.2,1+rand(rd),1> rotate z*teta translate PosOld rotate y*degrees(j/(abs(PosOld.x)))
                            texture{
                                pigment{rgb<C_Roof.x+rand(rd)*0.02,C_Roof.y+rand(rd)*0.02,C_Roof.z+rand(rd)*0.02>*(1+rand(rd))*0.2}
                                finish{ambient 0 diffuse 1 specular 0.1 roughness 0.02}
                            }
                        }
                        #declare j=j+xRFT;
                    #end                
                #end
                #declare PosOld=Pos;
                #declare i=i+1/n;
            #end    
        }
            
        union{ // support beams
            #declare i=0;
            #declare nBeam=40;
            #declare RoofBeam=superellipsoid{<0.3,0.3> scale <0.4,0.2,0.15>}
            #while (i<360)
                object{RoofBeam translate x*0.1 texture{T_Beam scale 0.3*<1,2,0.5> translate rand(rd)*5} rotate -z*55 translate <-rT,0,0> rotate y*i}
                #declare i=i+360/nBeam;
            #end
            texture{T_Beam scale 0.3}
        }
        union{ // spire
            cone{-y,rTRS*2.8,y*yTRS*0.1,rTRS}
            cone{0,rTRS,y*yTRS,rTRS*0.2}
            texture{T_Spire scale yTRS*1.02}
            translate y*yTR
        }
        translate y*yT
    }                                  
    union{ // secondary square tower
        #declare hfTR2=height_field{jpeg "hftowersq" texture{T_Wall} translate -y scale <1,0.05,1> rotate x*-90}
        object{mHFCorner(hfTR2,xT2,yT,xT2,0.1,5) translate <-xT2/2,0,-xT2/2>}        
        

        #declare xRFT=0.2;
        #declare yRFT=9/40;
        #declare eRFT=0.02;
        #declare rTRS=rTRS/2;
        #declare RoofTile=box{<-xRFT/2,-yRFT/2,-eRFT/2>,<xRFT/2,yRFT/2,eRFT/2>}
        #declare rd=seed(0);
        #macro mTowerRoofSQ(xP,xP2,yP)
        #local RoofTmp=
            union{
                #local iY=0;
                #local yPa=sqrt(yP*yP+((xP-xP2)*(xP-xP2))/4);
                #local xStart=0;
                #local xEnd=xP;
                #while (iY<yPa)
                    #local xStart=iY*((xP-xP2)/2)/yPa;
                    #local iX=xStart;
                    #local i=0;
                    #local xEnd=xP - iY*((xP-xP2)/2)/yPa;
                    #while (iX<=xEnd+xRFT)
                        #local Pos=iX*x+y*iY;
                        #local RoofTemp=
                        object{RoofTile scale <1+rand(rd)*0.1,1+rand(rd)*0.1,1> rotate x*rand(rd)*5 translate Pos
                            texture{
                                pigment{rgb<C_Roof.x+rand(rd)*0.02,C_Roof.y+rand(rd)*0.02,C_Roof.z+rand(rd)*0.02>*(1+rand(rd))*0.2}
                                finish{ambient 0 diffuse 1 specular 0.1 roughness 0.02}
                            }
                        }
                        #if (i=0)
                            object{RoofTemp clipped_by{plane{x,0 rotate -z*degrees(atan2((xP-xP2)/2,yPa)) inverse}}}
                        #else
                            #if (iX>xEnd-xRFT)
                                object{RoofTemp clipped_by{plane{x,0 rotate z*degrees(atan2((xP-xP2)/2,yPa)) translate x*xP}}}
                            #else
                                object{RoofTemp}
                            #end
                        #end
            
                        #local iX=iX+xRFT;
                        #local i=i+1;
                    #end
                    #local iY=iY+yRFT;    
                #end
                translate -x*xP/2
                rotate x*degrees(atan2((xP-xP2)/2,yP)) 
                translate -z*xP/2
                            
            }
            union{
                object{RoofTmp}
                object{RoofTmp rotate y*90}
                object{RoofTmp rotate y*180}
                object{RoofTmp rotate y*270}
            }
        #end         
        union{
            #declare xTR2b=xTR2*pow(0.8,2);
            #declare xTR2c=xTR2*pow(0.7,2);
            object{mTowerRoofSQ(xTR2,xTR2b,yTR2*0.2)}
            object{mTowerRoofSQ(xTR2b,xTR2c,yTR2*0.1) translate y*yTR2*0.2}
            object{mTowerRoofSQ(xTR2c,0.1,yTR2*0.7) translate y*yTR2*0.3}
            translate y*(yT+0.1)
        }
        union{ // spire
            cone{-y,rTRS*2.8,y*yTRS2*0.1,rTRS}
            cone{0,rTRS,y*yTRS2,rTRS*0.2}
            texture{T_Spire scale yTRS2*1.02}
            translate y*(yTR2+yT)
        }
        union{ // support beams
            #declare i=1;
            #declare nBeam=5;
            #while (i<=nBeam)
                object{RoofBeam translate x*0.6 rotate -z*55 rotate y*-90 translate <-xTR2/2,0,-xTR2/2> translate x*i*xTR2/(nBeam+1) texture{T_Beam scale 0.3*<0.1,2,1>}} //i*xTR2/nBeam}
                object{RoofBeam translate x*0.6 rotate -z*55 rotate y*180 translate <xTR2/2,0,-xTR2/2> translate z*i*xTR2/(nBeam+1) texture{T_Beam scale 0.3*<2,0.3,0.1>}}
                #declare i=i+1;
            #end
            texture{T_Beam scale 0.3}
            translate y*(yT+0.2)
        }

        translate x*rT*1.2
    }        
} 
//---------------------------
// Road and wall
//---------------------------              
#declare yStep=0.224;
#declare xStep=0.283;
#declare xRW1=4*xCren+xCren2;
#declare yRW1=5.62;
#declare zRW1=7.15;
#declare yPF1=yRW1-yCren; // platform height
#declare xPF1=6.75;  
#declare zPF1=10;
#declare zPF=1.5; // platform width
#declare yRW0=20;
#declare xLH=0.45; // small loophole width
#declare yLH=2.3*xLH; // small loophole height
#declare hfRW1PF1=height_field{jpeg "hfrw1pf1" water_level 0.01 rotate x*-90}
#declare hfTop=height_field{jpeg "gt11smallnb" translate -y*0.3 scale <1,0.05,1>}
#declare T_RW1PF1=texture{T_Wall} 
#declare T_RW1PF1a=texture{T_Wall}
#declare hfRW1PF1a=height_field{jpeg "hfrw1pf1a" texture{T_Wall} rotate -x*90 }
#declare T_Road=texture{
    pigment{
        gradient z turbulence 1 lambda 3
        triangle_wave
        color_map{
            [0.25 C_Stone0*0.8]
            [0.71 C_Stone0*0.7]
        }
    }
    finish{ambient 0 diffuse 1 specular 0 crand 0.1}
}        
#declare T_Road2=texture{
    pigment{
        gradient z turbulence 1 lambda 3
        triangle_wave
        color_map{
            [0.25 C_Stone0*0.7]
            [0.71 C_Stone0*0.5]
        }
    }
    finish{ambient 0 diffuse 1}
}        
#declare T_RWCrenel=texture{
        pigment{
            bozo
            turbulence 1
            lambda 3
            color_map{
                [0.5 C_Stone0]
                [0.8 C_Stone0*0.3]
            }
        }
        normal{bozo 2 scale 0.01}
        finish{ambient 0 diffuse 1}
        scale 0.3
    }
    texture{
        pigment{
            gradient y
            turbulence 0.3 omega 1 lambda 3
            poly_wave 1.5
            color_map{
                [0 rgbf <98/255,80/255,50/255,1>]
                [0.65 rgbf <98/255,80/255,50/255,3>*0.1]
                [1 rgb <98/255,80/255,50/255>*0.4]
            }
        }    
        finish{ambient 0 diffuse 1}
        scale (yCren+yStep)
    }
#declare T_RWStep=texture{
        pigment{
            bozo
            turbulence 1
            lambda 3
            color_map{
                [0.5 C_Stone0*0.8]
                [0.8 C_Stone0*0.4]
            }
        }
        normal{bozo 2 scale 0.01}
        finish{ambient 0 diffuse 1}
        scale 0.3
    }
    texture{
        pigment{
            gradient y
            turbulence 0.3 omega 1 lambda 3
            poly_wave 1.5
            color_map{
                [0 rgbf <98/255,80/255,50/255,1>]
                [0.65 rgbf <98/255,80/255,50/255,3>*0.1]
                [1 rgb <98/255,80/255,50/255>*0.3]
            }
        }    
        finish{ambient 0 diffuse 1}
        scale (yCren+yStep)
}
#declare T_RWPF=texture{
                        pigment{
                            bozo
                            turbulence 1
                            lambda 3
                            color_map{
                                [0.5 C_Stone0*0.8]
                                [0.8 C_Stone0*0.7]
                            }
                        }
                        normal{bozo 2 scale 0.01}
                        finish{ambient 0 diffuse 1}
                        scale 0.5
                    }
#declare e=0.05;
#declare n=0.05;                                  
#undef FPig1
#undef FPig2
#declare FPig1=function{pigment{brick color White, color Black turbulence 0.5 scale 0.2}}
#declare FPig2=function{pigment{crackle turbulence 0.1 translate 1}}
#declare RWBrick=isosurface {
	function { pow((pow(abs(x),2/e)+pow(abs(y),2/e)),e/n)+pow(abs(z),2/n)-1 + 0.5*FPig1(x*0.8,y*0.8,z*0.8).gray+ 1.1*FPig2(x*0.8,y*0.8,z*0.8).gray}
	contained_by { box { -1, 1 } }
    max_gradient 14
}               
#declare RWStep=isosurface {
	function { pow((pow(abs(x),2/e)+pow(abs(y),2/e)),e/n)+pow(abs(z),2/n)-1 + 0.2*FPig1(x*0.8,y*0.8,z*0.8).gray+ 0.8*FPig2(x*0.8,y*0.8,z*0.8).gray}
	contained_by { box { -1, 1 } }
    max_gradient 14
}               
#declare RW1=union{
    union{
        union{
            object{hfRW1PF1}
            object{hfRW1PF1 scale <1,1,-zCren*2>}
            texture{T_RW1PF1}
            scale <xLH*7,yCren,0.1>            
        }
        union{
            object{hfRW1PF1}
            object{hfRW1PF1 scale <1,1,-zCren*2>}
            texture{T_RW1PF1}
            scale <xLH*7,yCren,0.1>            
            translate x*xLH*6
        }
        rotate -y*90
        translate y*(yRW1-yCren)+z*zCren+x*zCren
    }        
    union{ // end platform
        box{0,<xPF1,yPF1,zPF1>}
        object{hfTop scale <xPF1,1,zPF1> translate y*yPF1}
        object{hfRW1PF1a scale <zPF1,yPF1,0.1> rotate y*-90 translate x*xPF1}
        texture{T_RWPF}
    }

    box{-y*yRW0,<xRW1,yPF1,zPF> translate z*zCren texture{T_RWPF}} // side platform
    object{hfTop scale <xRW1,0.5,zPF> translate z*zCren + y*yPF1 texture{T_Road}} // side platform
    union{
        box{-y*yRW0,<xRW1,yRW1-yCrenLH,zCren> texture{T_RW1PF1a scale <xRW1/3,yRW1,1>}} // side crenels
        

        union{
            object{RWBrick rotate -y*90 translate x+y+z scale 0.5 scale <xCren2,yCrenLH,zCren>}
            object{RWBrick rotate y*90 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*(xCren2+xCrenLH)} // crenel
            object{RWBrick translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*(xCren2+2*xCrenLH+xCren3)}
            object{RWBrick rotate y*180 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*(xCren2+3*xCrenLH+2*xCren3)}
            object{RWBrick rotate z*90 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*(xCren2+4*xCrenLH+3*xCren3)}
            translate y*(yRW1-yCrenLH)
            texture{T_RW1PF1a scale <xRW1/3,yRW1,1> rotate x*90 rotate z*45}
        }
        texture{T_Road}
    }
     

    #declare aRoad=7;
    #declare hfRoad=union{
        height_field{jpeg "gt11bignb"}
        box{<0,-1,0>,<1,0.3,1>}
    }        
    union{
        object{hfRoad scale <xRW1,0.05,zRW1*2>}
        object{hfRoad scale <xRW1*1.2,0.05,zRW1*2> rotate z*-aRoad translate x*xRW1}
        texture{T_Road scale <40,1,6>}
    }
}                     
                                     
#declare xRW2=2*xCren+xCren3;
#declare nStep=5;
#declare yRW2=yRW1-nStep*yStep;
#declare yPF2=yPF1-nStep*yStep;
#declare RW2=union{
    box{-y*yRW0,<xRW2,yRW2-yCrenLH,zCren>}
    union{
        object{RWBrick rotate y*90 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren>}
        object{RWBrick translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*xCren}
        object{RWBrick rotate y*180 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren> translate x*2*xCren}
        texture{T_RWCrenel}
        translate y*(yRW2-yCrenLH)
        

    }
    union{
        object{RWStep translate x+y+z scale 0.5 scale <xRW2,yPF2,zPF+xStep>
                texture{T_RWStep translate y*(-yCren+yPF2)}
        }
        union{ // steps
            #declare i=0;
            #while (i<nStep)                                               
                object{RWStep rotate y*90*i translate x+y+z scale 0.5 scale <xStep*(nStep-i),yStep*(i+1),zPF> texture{T_RWStep translate -y*yCren}}
                #declare i=i+1;
            #end
            translate y*yPF2+x*xStep
        }
        translate z*zCren
    }
}
#declare xRW3=xCren3+xCren;
#declare nStep=5;
#declare yRW3=yRW2-nStep*yStep;
#declare yPF3=yPF2-nStep*yStep;
#declare RW3=union{
    union{
        box{-y*yRW0,<xRW3,yRW3-yCrenLH,zCren>}
        union{                       
            object{RWBrick rotate y*90 translate x+y+z scale 0.5 scale <xCren3,yCrenLH,zCren>}
            object{RWBrick rotate y*90 translate x+y+z scale 0.5 scale <xCren,yCrenLH,zCren> translate x*xCren}
            texture{T_RWCrenel}
            translate y*(yRW3-yCrenLH)
        }
    }
    union{
        object{RWStep rotate y*90 translate x+y+z scale 0.5 scale <xRW3,yPF3,zPF+xStep>
                texture{T_RWStep translate y*(-yCren+yPF3)}
        }
        union{ // steps
            #declare i=0;
            #while (i<nStep)
                object{RWStep rotate y*90*i translate x+y+z scale 0.55 scale <xStep*(nStep-i),yStep*(i+1),zPF>
                texture{T_RWStep translate -y*(yCren-yStep*i)}
                }
                #declare i=i+1;
            #end
            translate y*yPF3+x*xStep
        }
        union{ // steps
        
            object{RWStep rotate y*90*i translate x+y+z scale 0.55 scale <xStep*2,yStep,zPF>}
            object{RWStep rotate y*90*i translate x+y+z scale 0.55 scale <xStep,yStep*2,zPF>}
            texture{T_RWStep translate -y*yCren}
            
            translate <xRW3,yPF3-3*yStep,0>
        }
        translate z*zCren
    }
}
#declare xRW4=xCren3+5*xCren;
#declare yRW4=yRW3-3*yStep;
#declare yPF4=yPF3-3*yStep;
#declare RW4=union{         
        #declare i=0;
        #while (i<5)
            union{
                union{
                    difference{
                        object{RWBrick rotate y*90*i translate x+y+z scale 0.55 scale <xCren3,yCren+yStep,zCren>}
                        box{<0,0,-0.1>,<xLH,yLH,zCren*1.1> translate <(xCren3-xLH)/2,-yLH/2+yCren/2,0>}
                        translate y*-yStep
                    }
                    box{-y*yRW0,<xCrenLH,yCren-yCrenLH,zCren> translate x*xCren3}
                    texture{T_RWCrenel translate x*i*xCren}
                }
                union{
                    object{RWStep rotate y*90*i translate x+y+z scale 0.55 scale <xCren*0.5,yPF3,zPF+xStep> translate -y*yPF3}
                    object{RWStep rotate y*i translate x+y+z scale 0.55 scale <xCren*0.5,yPF3,zPF+xStep> translate <xCren*0.5,-yStep-yPF3,0>}
                    texture{T_RWCrenel translate -y*yCren}
                    translate z*zCren
                }    
                translate x*i*xCren+y*(yPF4-i*yStep*2)
            }
            #declare i=i+1;
        #end                
        object{hfRW1PF1a scale <zPF1,yPF1,-0.1> translate <yStep*2,-7*yStep+yPF3-yPF1,zCren+zPF+xStep*1.5>}
}
#declare RoadWall=union{
    object{RW1}
    union{
        object{RW2}
        union{
            object{RW3}
            object{RW4 rotate y*20 translate x*xRW3}
            translate x*xRW2
        }
        rotate y*-3 
        translate <xRW1,0,0>
    }
    texture{T_Wall} 

}
//---------------------------
// Whole shebang
//---------------------------              
#declare BSet1=union{
    union{
        object{STower rotate y*15 translate <3,-5,12>}
        union{
            union{
                object{Wall1_1 scale <xB3/xW1,1,1>}
                object{Wall1_1 scale <xB3/xW1,1,1> rotate  -y*90 translate <zW,0,zW>}
                object{Build1 rotate y*90 translate <xB3-zB1,-1,xB1+0.3>}
                object{Build3 scale 1.5 rotate y*-15 translate <xB3-zB1-xB3a*1.5,yW1-0.5,0.5>}
                rotate x*1 translate -y*2
            }                
            union{
                object{Wall1_2 scale <-1,1,1> translate x*xW1 scale <xB4/xW1,1,1>} 
                object{Wall1_2 scale <-1,1,1> translate x*xW1 scale <xB4/xW1,1,1> rotate  -y*90 translate <zW,0,zW>} 
                rotate x*1 
                translate -y*8
                translate -x*xB4 rotate y*5 translate <-1,0,2>
            }
        }
        translate -x*xB3 
        rotate y*-10 
        translate -x*3
    }
    object{Build1}
    object{Build2 translate z*zB1}
    object{Build2 scale <1,1,-1> translate z*(zB1+zB2*2.5)}
    union{
        object{Wall1}
        object{Wall2 translate z*zR}
        object{Wall2 rotate -y*85 translate <xW1+rT,-4,zR+rT>}
        object{Tower rotate y*-25 translate <xW1+rT,0,zR+rT*0.5>}
        translate x*xB1
    }      
    object{TowerBody2 scale 0.5 translate <113,9,-16.5>} // close up tower
    object{RoadWall translate z*zW rotate y*15 translate <xB1+xW1-3,0,-zRW1>}
    texture{T_Wall}
}

// ----------------------------------------
// Mountain
// ----------------------------------------
#declare C_M1=rgb 0.25*<37,61,13>/255;
#declare C_M2=rgb 0.5*<106,105,35>/255;
#declare C_M3=rgb 0.25*<126,129,45>/255;
#declare T_Mountain=texture{T_Wall}

#declare Mountain=height_field{jpeg "hfsol3" water_level 0.01 translate <-0.5,0,-0.5> scale <205,50,200> 
    texture{T_Mountain} 
    translate -y*40
    translate x*2
}


// ----------------------------------------
// Stones
// ----------------------------------------
#declare C_Stone3=rgb<0.985,0.823,0.725>*0.2;
#declare C_Stone4=rgb<0.963,0.95,0.835>*0.2;

union{                 
    #declare rd=seed(2);
    #declare i=0;          
    #while (i<10)                                  
            #declare Start=<-rand(rd)*15-1,10,-50+rand(rd)*60>;
            #declare Dir=-y;
            #declare Norm1=<0,0,0>;
            #declare Inter=trace( Mountain, Start, Dir, Norm1);
            #undef Pig
            #declare Pig=function{
                pigment{
                    agate
                    color_map{[0 White][1 Black]}
                    translate rand(rd)*5
                    scale 0.8
                    }
            }
            #if (vlength(Norm1)!=0)   
                    isosurface{
                        function{x*(x+0.3)+y*(y+0.3)+z*(z+0.3) -1
                        +Pig(x,y,z).gray*0.5 
                        }
                        max_gradient 6
                        contained_by{box{-2,2}}
                    rotate 360*rand(rd)*y scale (0.5+rand(rd)*0.8)*2
                    mAlign(Inter,Inter+Norm1)
                    texture{pigment{bozo turbulence 1 scale 0.1*(1+rand(rd)) color_map{[0.3 color C_Stone0][0.6 color C_Stone1]}} normal{agate 0.5 scale 0.1} finish{ambient 0 diffuse 0.8}}
                    }
            #end
            #declare i=i+1;
    #end
}                                         

// ----------------------------------------
// Object placement
// ----------------------------------------
object{BSet1 rotate y*90}
object{Mountain}

// ----------------------------------------
// Sky
// ----------------------------------------
//background{White}
#declare C_Sky=rgb<104,129,209>/255;
sky_sphere{
    pigment{
        gradient y
        color_map { [0 C_Sky*2] [1 C_Sky]}
        scale 2
        translate y
    }
}

// ----------------------------------------
// Light
// ----------------------------------------
#declare C_Lum=rgb<253,242,220>*2/255;

light_source{
    x*100000
    color C_Lum*3.1
    rotate z*50
    rotate y*62
}                                          


// ----------------------------------------
// Camera
// ----------------------------------------
// See check comments to see the settings
#declare PdV1 = <0, 30, -130>;   // KFF20 rotate with clock*140 
#declare PdV2 = <0, 10, -70>;    // KFF20 rotate with clock*140

#declare PdV3 = <-40, 15, -90>;   // KFF8 translate with clock*300
#declare PdV4 = <-40, 25, -100>;   // KFF6 translate with clock*300

#declare PdV5 = <-35, 30, -100>;   // KFF10 translate with clock*120
#declare PdV6 = <-35, 30, -80>;   // KFF10 translate with clock*120


#declare V1 = <0, 15,  0>;
#declare V2 = <0, 15,  0>;
#declare V3 = <-10, 10, -90>;
#declare V4 = <-10, 10, -100>;
#declare V5 = <10, 20, -100>;
#declare V6 = <10, 20, -100>;


camera
{
  location  PdV6
  angle 90
  right <16/9,0,0>
  look_at   V6
  //rotate <0, clock*180, 0>
  translate <0, 0, 120*clock>
}


