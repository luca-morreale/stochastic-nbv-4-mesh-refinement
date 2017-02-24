// =========================================
// City building front
// -----------------------------------------
// Made for Persistence of vision 3.6
//==========================================  
// Copyright 2000-2004 Gilles Tran http://www.oyonale.com
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
//==========================================  
// This is the front of a turn-of-the-century, New-York style building
// There are so many parameters that I can't comment them...
// Basically the building is made of :
// A ground floor (in several parts)
// A series of middle floors
// A sub-roof floor (in several parts)
// A roof
// The building is assembled from several, 1-window wide "slices" going  from ground to floor
// The unit is roughly 1 pov unit = 1 metre
// =========================================

#include "colors.inc"       
global_settings{
    assumed_gamma 1
    radiosity{
//        count 200 error_bound 0.1
        recursion_limit 1
        brightness 1
        normal on
    }
}


// See check comments to see the settings
#declare PdV1 = <-40, 45, 40>;  // KFF20 clock*140
#declare PdV2 = <-40, 20, 40>;  // KFF20 clock*90
#declare PdV3 = <-30, 60, 30>;  // KFF20 clock*100
#declare PdV4 = <-35, 00, 35>;  // KFF20 clock*100

camera{
    look_at   <0, 30, 0>
    location PdV2
    angle 90
    rotate <0,clock*90,0>
    right <16/9,0,0>
}



// ---------------------------------------------------------------
// Sky, light etc.
// ---------------------------------------------------------------

#declare C_Light=rgb<243,199,107>/255;
light_source{<1,1,-1>*1000 color C_Light}
#declare C_SkyTop=rgb <243,246,252>/255;
#declare C_SkyBottom=rgb <159,182,201>/255;
#declare Sky=sky_sphere {
    pigment {
        gradient x
        scale <2,1,1>
        translate -x
        pigment_map{
            [0 color C_SkyBottom*0.6]
            [1
                function {min(1,max(0,y))}
                poly_wave 0.6
                color_map{
                    [0 color C_SkyBottom]
                    [1 color C_SkyTop]
                }
            ]
        }
    }
}

#declare T_Clouds = texture{
    pigment {
        pigment_pattern{
            gradient z
            triangle_wave
            octaves 7
            turbulence 1
            lambda 3
            scale 3
        }
        pigment_map {
            [0 color Clear]
            [0.6 bozo
                color_map {
                    [0.1 White]
                    [0.8 Clear]
                }
                scale 0.5
                turbulence 1
                octaves 6
                omega 0.7
                lambda 3
            ]
            [1 bozo
                color_map {
                    [0 White]
                    [0.4 Clear]
                }
                scale 0.5
                turbulence 1
                octaves 6
                omega 0.7
                lambda 2
            ]
            
        }
        octaves 6
        omega 0.7
        lambda 2
    }
    finish{ambient 1.1 diffuse 0}
}

sky_sphere{Sky}
// cloud layer
sphere{0,1
    scale <3,1,3>*10000
    texture{T_Clouds scale 30000 rotate y*90}
    hollow 
}

plane{y,0 texture{pigment{White*0.6}finish{ambient 0 diffuse 1}}}
// ---------------------------------------------------------------
// Building
// ---------------------------------------------------------------


#include "gt_winsmall.inc"
#declare T_Glass=texture{
    pigment{color rgbt <0.8,0.8,0.9,0.9>}       
    finish{ambient 0 diffuse 0.3 reflection 0.1}
}

#declare zB=30;      // Depth of the building
#declare yGF0=4.5;
#declare yGF1=4.5;
#declare yGF2=4;
#declare yFloor=3.4;
#declare nFloor=7;  // number of middle floors
#declare yLFloor=3.4;
#declare yRFloor=7;
#declare yRoof=1.5;
#declare xBi=5.8;
#declare nB=4;        // number of slices
//#declare xB=nB*xBi;   // x length of the building
#declare eB=0.4;
// ---------------------------------------------------------------
// Ground Floor 0
// ---------------------------------------------------------------
#declare xF=xBi*2/3;
#declare xP=xBi/3;
#declare yP1=1;
#declare yP2=yGF0;                
#declare zP1=0.3;
#declare zP3=0.3;
#declare zP2=0.5;
#declare yP3=1.5;
#declare yF=yGF0-yP1-yP3;            
#declare yLedgeGF=0.1;
#declare zLedgeGF=0.07;
#declare T_Frame=texture{
    pigment{rgb<0.2,0.9,0.7>*0.1}
    finish{ambient 0 diffuse 0.6 specular 0.1 roughness 0.05}
}              
#declare FenGF0=union{
    object{mWindow(1,xF*0.1,yF,0.05,0.08,0.025,0.025,0,2,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate x*xF*0.1}
    object{mWindow(1,xF*0.25,yF,0.05,0.08,0.025,0.025,0,1,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate x*xF*(0.2+0.25)}
    object{mWindow(1,xF*0.15,yF,0.05,0.08,0.025,0.025,0,2,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate x*xF*(0.2+0.5+0.15)}
}
#declare LedgeGF=union{
    box{0,1 scale <1,4/7,4/5>}
    box{0,1 scale <1,2/7,1>}
    difference{
        cylinder{0,x,2}
        cylinder{-x,x*1.1,1}
        plane{y,0 inverse}
        plane{z,0 inverse}
        scale <1,2/7,2/5>
        translate <0,6/7,4/5>
    }                       
    box{0,1 scale <1,1/7,1/5>}
    scale <xF,yLedgeGF,-zLedgeGF>
    
}
#declare P1=union{
    box{0,1 scale <xF,yP1,-zP1>}
    object{LedgeGF translate <0,yP1-yLedgeGF,-zP1>}
}
#declare P2=union{
    box{0,1 scale <xP,yP2,-zP2>}
    object{LedgeGF scale <0.5,1,1> translate <0,yP1-yLedgeGF,-zP2>}
}
#declare P3=union{
    box{0,1 scale <xF,yP3,-zP3>}
    object{LedgeGF scale <1,-1,1> translate <0,yLedgeGF,-zP3>}
}
#declare Bd1GF0=union{
    object{P1}
    object{P2 translate x*xF}
    object{FenGF0 translate y*yP1}
    object{P3 translate y*(yF+yP1)}
}        
#declare Bd1GF0Terminal=object{P2 translate -x*xP}

// ---------------------------------------------------------------
// Ground Floor 1
// ---------------------------------------------------------------
#declare Bd1GF1=box{0,<xBi,yGF1,zB>}
#declare yLedge0=yGF1*1.5/12.5;
#declare zLedge0=0.1;
#declare yLedge1=yGF1*1/12.5;
#declare zLedge1=0.2;

#declare yLedge2=yGF1*1/12.5;
#declare zLedge2=0.2;
#declare yFrise=yGF1*4/12.5;
#declare zFrise=0.1;
#declare yLedge3=yGF1*1.5/12.5;
#declare zLedge3=0.7;
#declare xLedge0=2*xBi/3;
#declare xF0=xLedge0/3;
#declare xF1=xBi/3;
#declare rColonne=yGF1*0.5/12.5;
#declare yColonne=yGF1-yLedge3-yFrise-yLedge0;
#declare yF1=yGF1-yLedge3-yFrise;
#declare FenGF1=object{mWindow(1,xF0*0.5,yColonne,0.05,0.08,0.025,0.025,0,0,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass)
    translate <xF0*0.5,0,0.1>
}

#declare Ledge0=difference{
    box{-z*3,1}
    box{<-0.1,0.2,0>,<1.1,0.8,1> translate z*0.9}
    box{<-0.1,0.3,0>,<1.1,0.7,1> translate z*0.7}
    scale <xLedge0,yLedge0,-zLedge0>
}

#declare Ledge1=union{
    box{0,<1,0.2,0.2>}
    cylinder{0,x,0.4 translate <0,0.6,0.2>}
    box{0,<1,0.2,0.8> translate <0,0.6,0>}
    box{0,<1,0.2,1> translate <0,0.8,0>}
    scale <xF1,yLedge1,-zLedge1>
}
#declare Ledge2=union{
    cylinder{0,y,1+0.2*zLedge2/rColonne}
    torus{1+0.2*zLedge2/rColonne,0.4*zLedge2/rColonne translate y*0.6}
    cylinder{0,y*0.2,1+0.8*zLedge2/rColonne translate y*0.6}
    cylinder{0,y*0.2,1+zLedge2/rColonne translate y*0.8}
    scale <rColonne,yLedge2,rColonne>
    
}
#declare Colonne=union{
    cylinder{0,y*yColonne,rColonne}
    box{<-rColonne-zLedge2,0,0>,<rColonne+zLedge2,yColonne,0.5>}
    object{Ledge2 translate y*(yColonne-yLedge2)}
}                                           
#declare Finger=union{
    difference{cylinder{0,y,1} cylinder{0,y,0.5 translate z*0.7}}
    difference{sphere{0,1} sphere{0,0.5 translate z*0.7} scale <1,xBi/60,1>}
}

#declare Ecusson=union{
    object{Finger translate -x*2}
    object{Finger translate x*2}
    object{Finger translate -y*0.1}
    scale <1/6,1,0.5>
    scale <xBi/10,1,1>
    translate <0,0,0.5>
}                           
#declare Frise=union{
    box{<0,0,-1/zFrise>,<xBi,1,0.3>}
    box{0,<xBi,0.7,0.5> translate y*0.3}
    #declare i=0;
    #while (i<5)
        union{
            object{Ecusson scale <1,0.7,1> translate y*0.3}
            object{Ecusson translate -y rotate z*180 rotate x*90 scale <1,xBi/60,zLedge3/zFrise> translate y}
            translate x*(i*2+1.5)*xBi/10
        }
        #declare i=i+1;
    #end
    scale <1,yFrise,-zFrise>
}
#declare FriseTerminal=union{
    box{<0,0,-1/zFrise>,<xF1,1,0.3>}
    box{0,<xF1,0.7,0.5> translate y*0.3}
    #declare i=0;
    #while (i<2)
        union{
            object{Ecusson scale <1,0.7,1> translate y*0.3}
            object{Ecusson translate -y rotate z*180 rotate x*90 scale <1,xBi/60,zLedge3/zFrise> translate y}
            translate x*(i*2+0.5)*xBi/10
        }
        #declare i=i+1;
    #end
    scale <1,yFrise,-zFrise>
}
#declare Ledge3=difference{
    box{0,1}
    box{<-0.1,0.2,0>,<1.1,0.8,1> translate z*0.95}
    box{<-0.1,0.3,0>,<1.1,0.7,1> translate z*0.8}
    scale <xBi,yLedge3,-zLedge3>
}
#declare Bd1GF1=union{
    object{Ledge0}
    union{
        box{0,<xF1,yF1,1>}
        object{Ledge1 translate y*(yF1-yLedge1)}
        translate x*xLedge0
    }
    union{
        object{Colonne translate x*xF0}
        object{Colonne translate 2*x*xF0}
        object{FenGF1 scale <(xF0-zLedge2-rColonne)/xF0,1,1>}
        object{FenGF1 scale <(xF0-zLedge2*2-rColonne*2)/xF0,1,1> translate x*(xF0+rColonne+zLedge2)}
        object{FenGF1 scale <(xF0-zLedge2-rColonne)/xF0,1,1> translate x*(2*xF0+rColonne+zLedge2)}
        translate y*yLedge0
        translate z*rColonne
    }                          
    union{
        object{Frise translate y*yF1}
        object{Ledge3 translate y*(yGF1-yLedge3)}
        translate -z*zFrise*2
    }
}
#declare Bd1GF1Terminal=union{
    union{
        box{0,<xF1,yF1,1>}
        object{Ledge1 translate y*(yF1-yLedge1)}
    }
    union{
        object{FriseTerminal translate y*yF1}
        object{Ledge3 scale <xF1/xBi,1,1> translate y*(yGF1-yLedge3)}
        translate -z*zFrise*2
    }
    translate -x*xF1
}
// ---------------------------------------------------------------
// Ground Floor 2
// ---------------------------------------------------------------
#declare Bd1GF2=box{0,<xBi,yGF2,zB> pigment{Cyan}}
#declare xF=(xBi/3)*0.8; 
#declare xM=xBi/3;
#declare yF=4*yGF2/7;
#declare TrouFenGF2=box{0,<xF,yF,3*eB> translate -z*eB}
#declare eC=0.2;
#declare CreuxGF2=box{0,<xM-2*eC,yF-2*eC,2> translate <eC,eC,-1>}
#declare FenGF2=union{
    object{mWindow(1,(xM*0.5-eC),yF*0.5,0.05,0.08,0.025,0.025,0,0,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass)}
    object{mWindow(1,(xM*0.5-eC),yF*0.5,0.05,0.08,0.025,0.025,0,0,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate <0,yF*0.5,-0.05>}
    translate <xM*0.5-eC,0,0.2>
}


#declare yLedge=(yGF2-yF)*0.8;
#declare zLedge=0.25;
#declare LedgeGF2=union{
    box{0,<1,0.1,0.1>}        
    box{0,<1,0.2,0.2> translate y*0.1}        
    cylinder{0,x,0.1 translate <0,0.4,0.2>}
    union{
        cylinder{0,x,0.1 translate <0,0.5,0.3>}
        difference{box{0,<1,0.2,0.2>} cylinder{-x*0.01,x*1.01,0.2 translate z*0.2} translate <0,0.5,0.4>}
        cylinder{0,x,0.1 translate <0,0.8,0.6>}
        box{0,<1,0.1,0.7> translate <0,0.8,0>}
        box{0,<1,0.1,0.9> translate <0,0.9,0>}
        translate -z*0.3
        scale <1,1,3>
        translate z*0.3
    }
    scale <xBi,yLedge,zLedge>
    scale <1,1,-1>
}
#declare Bd1GF2=union{
    object{FenGF2}
    object{FenGF2 translate x*(xBi/3)*1.1}
    difference{
        box{0,<xBi,yGF2,eB>}                                
        object{CreuxGF2 scale <1,1,eC*0.3> translate x*xBi*2/3}
        object{TrouFenGF2 translate -x*0.00001}
        object{TrouFenGF2 translate x*(xBi/3)*1.1}
    }                                        
    union{
        cylinder{0,x*(xM-2*eC),eC}
        cylinder{0,x*(xM-2*eC),eC translate y*(yF-2*eC)}
        cylinder{0,y*(yF-2*eC),eC}
        cylinder{0,y*(yF-2*eC),eC translate x*(xM-2*eC)}
        scale <1,1,0.3>
        translate <eC,eC,eC*0.5>
        translate  x*xBi*2/3
    }
    object{LedgeGF2 translate y*(yGF2-yLedge)}
}

#declare Bd1GF2Terminal=union{
    difference{
        box{0,<xBi/3,yGF2,eB>}                                
        object{CreuxGF2 scale <1,1,eC*0.3>}
    }                                        
    union{
        cylinder{0,x*(xM-2*eC),eC}
        cylinder{0,x*(xM-2*eC),eC translate y*(yF-2*eC)}
        cylinder{0,y*(yF-2*eC),eC}
        cylinder{0,y*(yF-2*eC),eC translate x*(xM-2*eC)}
        scale <1,1,0.3>
        translate <eC,eC,eC*0.5>
    }
    object{LedgeGF2 scale <1/3,1,1> translate y*(yGF2-yLedge)}
    translate -x*xBi/3
}

// ---------------------------------------------------------------
// Middle Floor
// ---------------------------------------------------------------
#declare Bd1Floor=box{0,<xBi,yFloor*0.8,zB>}

#declare xF=(xBi/3)*0.8; 
#declare yF=4*yFloor/7;                              
#declare TrouFenFloor=box{0,<xF,yF,3*eB> translate -z*eB}
#declare TrouFenFloor=union{
    box{-z,<xF,yF,3*eB>}
    cylinder{0,y*(yF-0.03),0.03}
    cylinder{0,y*(yF-0.03),0.03 translate x*xF}
    sphere{0,0.03 translate y*(yF-0.03)}
    sphere{0,0.03 translate y*(yF-0.03) translate x*xF}
}        
#declare FenFloor=union{
    object{mWindow(1,xF*0.5,yF*0.5,0.05,0.08,0.025,0.025,0,0,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass)}
    object{mWindow(1,xF*0.5,yF*0.5,0.05,0.08,0.025,0.025,0,0,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate <0,yF*0.5,-0.05>}
    translate <xF*0.5,0,0.2>
    scale <1,(yF-xF*0.05)/yF,1>
    translate y*xF*0.05
}

#declare LedgeFloor=box{-z*0.3,<xF,xF*0.05,xF*0.05> scale <1,1,-1>} 
#declare Bd1Floor=union{
    object{LedgeFloor}
    object{FenFloor}
    object{FenFloor translate x*(xBi/3)*1.1}
    object{LedgeFloor translate x*(xBi/3)*1.1}
    difference{
        box{0,<xBi,yFloor,eB>}
        object{TrouFenFloor translate -x*0.00001}
        object{TrouFenFloor translate x*(xBi/3)*1.1}
    }
}
#declare Bd1FloorTerminal=union{              
    #declare yBrick=0.8*yFloor/8;     
    #declare xBrick=0.6*xBi/3;
    #declare zBrick=0.05;
    #declare Brick=box{-z*5,1 scale <xBrick,yBrick,-zBrick>}
    #declare i=0;
    #while (i<4)
        union{           
            object{Brick}
            object{Brick scale <0.7,1,1> translate y*yFloor/8}
            translate <-zBrick*2,yFloor*i/4,0>
        }
    #declare i=i+1;
    #end
    box{0,<xBi/3,yFloor,eB>}
    translate -x*xBi/3
}
// ---------------------------------------------------------------
// Last Floor
// ---------------------------------------------------------------
#declare Bd1LFloor=box{0,<xBi,yLFloor,zB>}
#declare yLedge=(yFloor-yF)*0.7;
#declare zLedge=0.1;
#declare LedgeLastFloorBottom=union{
    box{0,<xBi,yLedge*0.5,zLedge*0.5>}

    difference{
        box{<0,-1,0>,<1,1,1>}
        cylinder{-x*0.1,x*1.1,0.8 scale <1,1,0.6> translate z}
        translate y
        scale <xBi,yLedge*0.25,zLedge>
        translate y*yLedge*0.5
    }
    
    scale <1,1,-1>        
    translate -y*yLedge
}
#declare yLedge=(yLFloor-yF)*0.9;
#declare zLedge=0.3;
#declare LedgeLastFloorTop=difference{
        box{<0,-1,0>,<1,1,1>}
        cylinder{-x*0.1,x*1.1,0.7 scale <1,1,0.9> translate z}
        box{<-0.1,-0.8,0>,<1.1,0.8,1> translate z*0.5}
        translate y
        scale <xBi,yLedge*0.5,zLedge>
        scale <1,1,-1>
    }
#declare Bd1LFloor=union{
    object{Bd1Floor scale <1,yLFloor/yFloor,1> translate z*0.1}
    object{LedgeLastFloorBottom}
    object{LedgeLastFloorTop translate <0,yLFloor-yLedge,0.1>}
}
#declare Bd1LFloorTerminal=union{              
    #declare i=0;
    union{
        object{LedgeLastFloorBottom translate -z*0.05}
        object{LedgeLastFloorTop translate <0,yLFloor-yLedge,0.1>}
        scale <1/3,1,1>
    }
    union{
        object{LedgeLastFloorBottom translate -z*0.05}
        object{LedgeLastFloorTop translate <0,yLFloor-yLedge,0.1>}
        scale <-0.15/xBi,1,1>
    }
    #while (i<2)
        union{           
            object{Brick}
            object{Brick scale <0.7,1,1> translate y*yFloor/8}
            translate <-zBrick*2,yFloor*i/4,0>
        }
    #declare i=i+1;
    #end
    object{Brick translate <-zBrick*2,yFloor*2/4,0>}
    box{0,<xBi/3,yFloor,eB>}
    translate -x*xBi/3
}
// ---------------------------------------------------------------
// Roof Floor
// ---------------------------------------------------------------
#declare Bd1RFloor=box{0,<xBi,yRFloor,zB>} 
#declare xF=2*xBi/3;
#declare xP1=xBi/3;
#declare xP2=xP1;
#declare yF1=xF/2;       
#declare yL1=yF1*0.4;
#declare rF2=xF/2;
#declare eP1=xP1/6;               
#declare eF=eP1*0.7;
#declare yP1=2*yF1+2*yL1;
#declare yP2=rF2+2*eP1;           
#declare zP1=0.2;                     
#declare zL1=0.15;
#declare yRFloor=yP1+yP2;
#declare FenRF0=object{mWindow(1,xF/6,yF1,0.05,0.08,0.025,0.025,0,2,0,0,0.12,0.025,T_Frame,T_Frame,T_Glass) translate x*xF/6}
#declare FenRF1=union{
    object{FenRF0}
    object{FenRF0 translate x*2*xF/6}
    object{FenRF0 translate x*4*xF/6}
    scale <(xF-2*eF)/xF,1,1>
    translate <eF,0,0.2>
}
#declare FenRF2=union{
    object{FenRF1}
    difference{
        cylinder{0,z*0.05,xF*0.5}
        cylinder{-z,z,xF*0.5-eF-0.05}
        plane{y,0}
        translate <xF*0.5+eF,0,0.1>
        texture{T_Frame}
    }
}        
#declare L1=union{
    box{-z*2,1 scale <1,1,0.5>}
    box{-z*2,1 scale <1,0.2,0.7>}
    box{-z*2,1 scale <1,0.4,0.7> translate y*0.3}
    box{-z*2,1 scale <1,0.2,0.7> translate y*0.8}
    scale <xF,yL1,-zL1>
}
#declare P1=union{
    #declare yP1a=yP1-yL1*0.3;
    box{-z*2,1 scale <xP1,yP1,-zP1*0.5>}
    box{-z*2,1 scale <xP1,yL1*0.2,-zP1> translate y*(yP1-yL1*0.2)}
    box{-z*2,1 scale <eP1,yP1a,-zP1>}
    box{-z*2,1 scale <-eP1,yP1a,-zP1> translate x*xP1}
    box{-z*2,1 scale <xP1,eP1,-zP1>}
    box{-z*2,1 scale <xP1,eP1,-zP1> translate y*(yP1a-eP1)}
    
}
#declare P2=difference{
    box{-z*2,1 scale <xBi,yP2,-zP1>}
    cylinder{-zP1*2*z,z*2,rF2 translate x*rF2}
    
}                                                   
#declare P2=union{
    box{0,1 scale <xP1,eP1,-zP1> translate <xF,yL1*0.1>}
    difference{
        cylinder{-zP1*z*0.5,z*2*zP1,rF2}
        cylinder{-zP1*z*2,z*3*zP1,rF2-eF}
        plane{y,0}                    
        translate x*rF2
    }                     

    box{<0,0,-zP1>,<eP1,eP1*2,zP1*3> rotate z*20 translate <rF2-eP1*2,yP2-2*eP1,0>}
    box{<0,0,-zP1>,<eP1,eP1*2,zP1*3> rotate z*20 scale <-1,1,1> translate <rF2+eP1*2.1,yP2-2*eP1,0>}
    union{
        union{
            sphere{0,1 scale <eP1*0.6,eP1*0.7,zP1> rotate z*40 translate <-eP1*0.8,0,0>}
            sphere{0,1 scale <eP1*0.6,eP1*0.9,zP1> }
            sphere{0,1 scale <eP1*0.6,eP1*0.7,zP1> rotate -z*40 translate <eP1*0.8,0,0>}
            translate <rF2,yP2-eP1*0.5,0>
        }                        
        union{
            sphere{0,1 scale <eP1*0.6,eP1*0.7,zP1> rotate z*40 translate <-eP1*0.8,0,0>}
            sphere{0,1 scale <eP1*0.6,eP1*0.9,zP1>}
            sphere{0,1 scale <eP1*0.6,eP1*0.7,zP1> rotate -z*40 translate <eP1*0.8,0,0>}
            scale 0.8
            translate <rF2,yP2-eP1*1.8,0>
        }                        
    }
    difference{
        union{
            box{0,1 scale <xBi,-eP1,-zP1> translate y*yP2}
            cylinder{-zP1*z,z*2*zP1,rF2+eP1 translate x*rF2}
        }
        cylinder{-zP1*z*2,z*3*zP1,rF2 translate x*rF2}
        union{
            box{<-eP1,-eP1,-zP1*2>,<eP1,eP1*3,zP1*3> scale <-1,1,1> rotate z*20}
            box{<-eP1,-eP1,-zP1*2>,<eP1,eP1*3,zP1*3> rotate -z*20}
            translate <rF2,yP2-2*eP1,0>
        }
        plane{y,yL1*0.1}                    
    } 
    
    difference{
        box{-z*2,1 scale <xBi,yP2,-zP1*0.5>}
        cylinder{-zP1*2*z,z*2,rF2 translate x*rF2}
    }
    
}
#declare P2Terminal=union{
    box{-z*2,1 scale <xP1,yP2,-zP1*0.5>}
    union{
        box{-z*2,1 scale <xP1,eP1,-zP1>}
        box{-z*2,1 scale <eP1,yP2-yL1*0.1,-zP1>}
        translate y*yL1*0.1
    }
    box{-z*2,1 scale <xP1,eP1,-zP1> translate y*(yP2-eP1)}
    
}
#declare Bd1RFloor=union{
    object{FenRF1}
    object{FenRF1 translate y*(yF1+yL1)}
    object{FenRF2 translate y*yP1}
    box{-z*2,1 scale <eF,yP1,-zP1*0.5>}
    box{-z*2,1 scale <-eF,yP1,-zP1*0.5> translate x*xF}
    object{P1 translate x*xF}
    object{L1 translate y*yF1}
    object{L1 translate y*(2*yF1+yL1)}
    object{P2 translate y*yP1}
}
#declare Bd1RFloorTerminal=union{
    object{P1}
    object{P2Terminal translate y*yP1}
    translate -x*xP1
}


// ---------------------------------------------------------------
// Roof 
// ---------------------------------------------------------------
#declare Bd1Roof=box{0,<xBi,yRoof,zB>}
#declare yFrise1=0.25;
#declare zFrise1=0.1;

#declare nF1=18;
#declare xBrickF1=xBi/nF1;
#declare yBrickF1=yFrise1;
#declare BrickF1=union{
    difference{
        box{0,1}
        cylinder{-z*0.1,z*0.1,0.4 translate <0.5,0.5,1>}
    }
    sphere{0,0.2 translate <0.5,0.5,1>}
    scale <xBrickF1*0.8,yBrickF1*0.8,-zFrise1> translate x*xBrickF1*0.1
}
    
#declare Frise1=union{
    box{0,1 scale <xBi,yFrise1,1>}
    #declare i=0;
    #while (i<nF1)
        object{BrickF1 translate x*i*xBrickF1}
    #declare i=i+1;
    #end
}
#declare Frise1Terminal=union{
    box{0,1 scale <xBi,yFrise1,1>}
    #declare i=0;
    #while (i<nF1/3)
        object{BrickF1 translate x*i*xBrickF1}
    #declare i=i+1;
    #end
}

#declare yFrise2=0.3;
#declare zFrise2=0.2;
#declare nF2=15;
#declare xBrickF2=xBi/nF2;
#declare yBrickF2=yFrise2;
#declare BrickF2=union{
    difference{
        box{0,1}
        cylinder{-z*0.1,z*0.1,0.4 translate <0.5,0.5,1>}
    }
    sphere{0,0.2 translate <0.5,0.5,1>}
    scale <xBrickF2*0.8,yBrickF2*0.8,-zFrise2> translate x*xBrickF2*0.1
}
#declare Frise2=union{
    box{0,1 scale <xBi,yFrise2,1>}
    #declare i=0;
    #while (i<nF2)
        object{BrickF2 translate x*i*xBrickF2}
    #declare i=i+1;
    #end
}
#declare Frise2Terminal=union{
    box{0,1 scale <xBi,yFrise2,1>}
    #declare i=0;
    #while (i<nF2/3)
        object{BrickF2 translate x*i*xBrickF2}
    #declare i=i+1;
    #end
}

#declare yFrise3=0.45;
#declare zFrise3=0.6;
#declare nF3=6;
#declare xBrickF3=xBi/nF3;
#declare yBrickF3=yFrise3;
#declare BrickF3=union{
    difference{
        cylinder{0,x,2}
        cylinder{-x,x*1.1,0.99}
        plane{y,0}
        plane{z,0 inverse}
        scale <1,2/8,2/8>
        translate <0,0,4/8>
    }                       
    box{0,1 scale <1,6/8,4/8> translate <0,2/8,0>}
    cylinder{0,x,3/8 translate <0,7/8,4/8>}
    box{0,1 scale <1,1/8,1> translate <0,7/8,0>}
    difference{
        sphere{0,1}
        cylinder{-z*1.1,z*1.1,0.5}
        scale <1,1,0.4>*2/8 translate z*3/8 rotate x*45   translate <0.5,7/8,4/8>
    }
            
    scale <xBrickF3*0.5,yBrickF3,-zFrise3>
}
#declare Frise3=union{
    box{0,1 scale <xBi,yFrise3,1>}
    #declare i=0;
    #while (i<nF3)
        union{
            object{BrickF3 scale <0.1,1.2,1.2>}
            object{BrickF3}
            object{BrickF3 scale <-0.1,1.2,1.2> translate x*xBrickF3*0.5}
            translate x*i*xBrickF3
        }
    #declare i=i+1;
    #end
}
#declare Frise3Terminal=union{
    box{0,1 scale <xBi,yFrise3,1>}
    #declare i=0;
    #while (i<nF3/3)
        union{
            object{BrickF3 scale <0.1,1.2,1.2>}
            object{BrickF3}
            object{BrickF3 scale <-0.1,1.2,1.2> translate x*xBrickF3*0.5}
            translate x*i*xBrickF3
        }
    #declare i=i+1;
    #end
}


#declare yLedgeR=0.7;
#declare zLedgeR=1.2;

#declare LedgeR=union{
    box{0,1 scale <xBi,yLedgeR,-zLedgeR*0.5>}
    union{
        box{0,1 scale <1,1/7,2/7>}
        box{0,1 scale <1,6/7,3/7> translate y*1/7}
        difference{
            cylinder{0,x,1.5}
            cylinder{-x,x*1.1,0.99}
            plane{y,0}
            plane{z,0 inverse}
            scale <1,3/7,3/7>
            translate <0,1/7,6/7>
        }
        box{0,1 scale <1,1/7,6/7> translate y*4/7}
        box{0,1 scale <1,2/7,1> translate y*5/7}
        scale <xBi,yLedgeR,-zLedgeR*0.5> 
        translate -z*zLedgeR*0.5
    }
}

#declare xBal=xBi*2/3;
#declare yBal=1.7;
#declare zBal=0.15;

#declare yPBal=yBal*0.7;
#declare xPBal=xBi/3;
#declare zLedgePBal=zBal*2;
#declare zLedgeBal=zBal*1.5;
#declare LedgePBal=union{
    box{0,1 scale <1,1/7,2/7>}
    box{0,1 scale <1,6/7,3/7> translate y*1/7}
    difference{
        cylinder{0,x,1.5}
        cylinder{-x,x*1.1,0.99}
        plane{y,0}
        plane{z,0 inverse}
        scale <1,3/7,3/7>
        translate <0,1/7,6/7>
    }
    box{0,1 scale <1,1/7,6/7> translate y*4/7}
    box{0,1 scale <1,2/7,1> translate y*5/7}
    scale <xPBal,yBal*0.3,-zLedgePBal> 
}

#declare PBal=union{
    box{-z*3,1 scale <xPBal,yPBal,-zBal*0.5>}
    box{0,1 scale <eP1,yPBal,-zBal>}
    box{0,1 scale <-eP1,yPBal,-zBal> translate x*xPBal}
    box{0,1 scale <xPBal,eP1,-zBal>}
    box{0,1 scale <xPBal,-eP1,-zBal> translate y*yPBal}
    object{LedgePBal translate y*yBal*0.7}
    
}             

#declare LedgeBal=union{
    box{0,1 scale <1,1/7,2/7>}
    box{0,1 scale <1,6/7,3/7> translate y*1/7}
    difference{
        cylinder{0,x,1.5}
        cylinder{-x,x*1.1,0.99}
        plane{y,0}
        plane{z,0 inverse}
        scale <1,3/7,3/7>
        translate <0,1/7,6/7>
    }
    box{0,1 scale <1,1/7,6/7> translate y*4/7}
    box{0,1 scale <1,2/7,1> translate y*5/7}
    scale <xPBal*2,yBal*0.3,-zLedgeBal> 
}
                      
#declare ColonneBal =lathe{
    cubic_spline
    12,<0.010,0.0>,<0.010,0.0>,<0.023,0.050>,<0.018,0.070>,	
    <0.029,0.092>,<0.033,0.126>,<0.017,0.149>,<0.023,0.164>,<0.025,0.176>,<0.030,0.200>,
    <0.020,0.230>,<0.020,0.250> 
    scale <1/0.04,1/0.23,1/0.04>
    scale <1,-1,1>
    translate y
}
                      
#declare Balustrade=union{
    object{PBal translate x*xBal}
    union{
        object{LedgeBal}
        box{0,1 scale <xBal,yBal*0.3,eP1>}
        translate y*yBal*0.7
    }
    #declare i=0;
    #declare nCol=7;
    #declare rCol=eP1*0.5;
    #declare riCol=(xBal/(nCol*2))-rCol;
    #while (i<nCol)
        //cylinder{0,y*yBal*0.7,rCol 
        object{ColonneBal
        scale <rCol,yPBal,rCol>
        translate <(rCol+riCol)*(2*i+1),0,eP1*0.3>
        }
        #declare i=i+1;
    #end                
    translate -z*0.7
    
}

#declare Bd1Roof=union{
    object{Frise1}
    object{Frise2 translate y*yFrise1}
    object{Frise3 translate y*(yFrise1+yFrise2)}
    object{LedgeR translate y*(yFrise1+yFrise2+yFrise3)}
    object{Balustrade translate y*(yFrise1+yFrise2+yFrise3+yLedgeR)}
}

#declare Bd1RoofTerminal=union{
    object{Frise1Terminal}
    object{Frise2Terminal translate y*yFrise1}
    object{Frise3Terminal translate y*(yFrise1+yFrise2)}
    object{LedgeR scale <xPBal/xBi,1,1> translate y*(yFrise1+yFrise2+yFrise3)}
    object{PBal translate -z*0.5 translate y*(yFrise1+yFrise2+yFrise3+yLedgeR)}
    translate -x*xPBal
}

// ---------------------------------------------------------------
// Single column
// ---------------------------------------------------------------
#declare Bd1i=union{
    object{Bd1GF0 translate z*0.5}
    object{Bd1GF1 translate y*yGF0}
    object{Bd1GF2 translate y*(yGF0+yGF1)}
    union{
        #declare i=0;
            #while (i<nFloor)
            object{Bd1Floor translate y*i*yFloor}
            #declare i=i+1;
        #end
        translate y*(yGF0+yGF1+yGF2)        
    }
    object{Bd1LFloor translate y*(yGF0+yGF1+yGF2+nFloor*yFloor)}
    object{Bd1RFloor translate y*(yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor)}
    object{Bd1Roof translate y*(yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor+yRFloor)}
}
// ---------------------------------------------------------------
// Single column terminal
// ---------------------------------------------------------------
#declare Bd1iTerminal=union{
    object{Bd1GF0Terminal translate z*0.5}
    object{Bd1GF1Terminal translate y*yGF0}
    object{Bd1GF2Terminal translate y*(yGF0+yGF1)}
    union{
        #declare i=0;
            #while (i<nFloor)
                object{Bd1FloorTerminal translate y*i*yFloor}
            #declare i=i+1;
        #end
        translate y*(yGF0+yGF1+yGF2)        
    }
    object{Bd1LFloorTerminal translate y*(yGF0+yGF1+yGF2+nFloor*yFloor)}
    object{Bd1RFloorTerminal translate y*(yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor)}
    object{Bd1RoofTerminal translate y*(yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor+yRFloor)}
}                                                                      
// ---------------------------------------------------------------
// Whole building
// ---------------------------------------------------------------
#declare xBuild1=xBi/3+nB*xBi; // building x size
#declare yBuild1=yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor+yRFloor+yFrise1+yFrise2+yFrise3+yLedgeR+yBal; // building height
#declare Building=union{
    box{<1.2,0,0>,<xBi*3.8+xBi/3,yGF0+yGF1+yGF2+nFloor*yFloor+yLFloor+yRFloor,1>                                           
        texture{
            pigment{
                cells
                color_map{
                    [0 color White*0.01]
                    [1 color White*0.5]
                }
            }
            finish{ambient 0.4 diffuse 0.8}                       
        }       
        translate z
    }
    object{Bd1iTerminal translate x*xBi/3} // ends the building side
    #declare i=0;  
    #while (i<nB)
        object{Bd1i translate x*xBi*i translate x*xBi/3} // add the slices
        #declare i=i+1;
    #end               
    texture{
        pigment{image_map{jpeg "buildmap"}} //
//                pigment{White}
        finish{ambient 0 specular 0.01 roughness 0.05 diffuse 0.9}
            scale <xBuild1,yBuild1,1>
    }
}

object{Building translate <-14,0,-14>}
object{Building rotate y*90 translate <-14,0,11>}
object{Building rotate y*270 translate <11,0,-14>}
object{Building rotate y*180 translate <11,0,11>}


#include "colors.inc"
#include "textures.inc"
#include "metals.inc"

#include "i_pillar.inc"
#declare r_pil=seed(363);

// *** rust ***
#declare t_rusty=
 texture{
  pigment{
   granite
   color_map{
    [0 DarkTan*.5]
    [0.5 DarkBrown*.5]
    [1 Orange*.6]
   }
  }
  normal{granite scale .1}
  finish{F_MetalA}
 }
 // *** pillar metal ***
#declare t_metal_pillar=
 texture{
  pigment{
   wrinkles
   turbulence .3
   color_map{
    [0.00 DkGreenCopper*.1]
    [1.00 Copper]
   }
  }
  normal{crackle .1 scale .5}
  finish{Dull metallic}
 }

// *** pillar metal ***
#declare t_pillar_metal=
texture{
 wrinkles
 turbulence .3
 texture_map{
  [0.00 t_rusty]
  [0.30 t_rusty]
  [0.32 t_metal_pillar]
  [1.00 t_metal_pillar]
 }
 scale 0.5
}
object{pillar(t_pillar_metal,rand(r_pil)) translate <15, 0, 20>}
