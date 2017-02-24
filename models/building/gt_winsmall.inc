//=========================================
// Window generator macro
// Stripped down version for the IRTC "City" round
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
// This file contains 1 macros to build windows
// mWindow: window panes and wood frame
//=========================================
// mWindow macro
//-----------------------------------------
// It creates an openable window
// The window is on the x-y plane and opens toward -z
//-----------------------------------------
//  !-------------------! yF
//  ! !--!!--! !------! !
//  ! !  !!  ! !  !!  ! !
//  ! !__!!__! !__!!__! !
//  ! !--!!--! !--!!--! !
//  ! !  !!  ! !  !!  ! !
//  ! !__!!__! !__!!__! !
//  ! !--!!--! !--!!--! !
//  ! !  !!  ! !  !!  ! !
//  ! !  !!  ! !  !!  ! !
//  ! !--!!--! !------! !
//  !-------------------!
//  -xF       0         xF
//  Window with 2 panes (typeF=2), 1 vertical bar (nFx=1) and 2 horizontal bars (nFy=2)
//-----------------------------------------
/*
#declare typeF=2;       // window type : 1= 1 pane ; 2= 2 panes
#declare xF1=3;         // Width of the half-window (pane width if 2 panes) -xF to xF
#declare yF=9;          // Height of window
#declare zF=0.33;       // Thickness of window
#declare eF1=0.2;       // Thickness of wood frame
#declare eF2=0.05;      // Thickness of wood frame (inner 1)
#declare eF3=0.05;      // Thickness of wood frame (inner 2)
#declare nFx=1;         // Number of vertical bars (pane separators)
#declare nFy=2;         // Number of horizontal bars (pane separators)
#declare tetaFG=40;     // angle of left pane (0=closed pane; 
#declare tetaFD=0;      // angle panneaux droite
#declare yGond=0.4;     // height of half hinge
#declare rGond=0.07;    // hinge radius
#declare txtF1=texture{pigment{Green}} // wood frame texture
#declare txtF2=texture{pigment{White}} // inner wood frame & bars texture 
#declare txtV=texture{pigment{rgbt<0.9,0.8,0.2,0.2>}} // glass texture
*/
//-----------------------------------------
//-----------------------------------------
// Type 1 = one pane
//-----------------------------------------
// closed
//#declare Window_1_c=object{mWindow(1,xL,yL,0.2,0.3,0.1,0.1,0,2,0,0,0.5,0.1,txtFen1,txtFen2,txtVit1)}
// part open
//#declare Window_1_o=object{mWindow(1,xL,yL,0.2,0.3,0.1,0.1,0,2,-120,-110,0.5,0.1,txtFen1,txtFen2,txtVit1)}
//-----------------------------------------
// Type 2 = two panes
//-----------------------------------------
// closed, 0 vertical bars, 2 horizontal bars
//#declare Window_2_c=object{mWindow(2,xL,yL,0.2,0.3,0.1,0.1,0,2,0,0,0.5,0.1,txtFen1,txtFen2,txtVit1)}
// part open, 0 vertical bars, 2 horizontal bars
//#declare Window_2_o=object{mWindow(2,xL,yL,0.2,0.3,0.1,0.1,0,2,-120,-110,0.5,0.1,txtFen1,txtFen2,txtVit1)}
// closed, 1 vertical bars, 3 horizontal bars
//#declare Window_3_c=object{mWindow(2,xL,yL,0.2,0.3,0.1,0.1,1,3,0,0,0.5,0.1,txtFen1,txtFen2,txtVit1)}
// part open, 1 vertical bars, 3 horizontal bars
//#declare Window_3_o=object{mWindow(2,xL,yL,0.2,0.3,0.1,0.1,1,3,-130,-150,0.5,0.1,txtFen1,txtFen2,txtVit1)}
//-----------------------------------------

#macro mWindow(typeF,xF1,yF,zF,eF1,eF2,eF3,nFx,nFy,tetaFG,tetaFD,yGond,rGond,txtF1,txtF2,txtV)
#if (typeF=1) // One pane
        #local xF=xF1*2;
#else         // Two panes
        #local xF=xF1;
#end

//-----------------------------------------
// Hinge
//-----------------------------------------
/*
#local Gonds=union{
        union{
                union{cylinder{0,y*yGond,rGond} sphere{y*yGond,rGond} translate y*rGond*0.2}
                union{cylinder{0,-y*yGond,rGond} sphere{-y*yGond,rGond} translate -y*rGond*0.2}
                translate y*yF/4
        }
        union{
                union{cylinder{0,y*yGond,rGond} sphere{y*yGond,rGond} translate y*rGond*0.2}
                union{cylinder{0,-y*yGond,rGond} sphere{-y*yGond,rGond} translate -y*rGond*0.2}
                translate y*3*yF/4
        }
} 
*/          
//-----------------------------------------
// Creates the window pane
//-----------------------------------------
#local Battant=union{
        // Glass
                union{
                        triangle{<0,0,0>,<xF,0,0>,<xF,yF,0>}
                        triangle{<0,0,0>,<0,yF,0>,<xF,yF,0>}
                        translate z*eF2*2
                        texture{txtV}
                }      
        // Frame
                difference{
                        box{<0,0,0>,<xF,yF,zF>}
                        box{<eF1,eF1,-1>,<xF-eF1,yF-eF1,zF*3>}
                }              
                difference{
                        box{<eF1,eF1,eF2>,<xF-eF1,yF-eF1,zF>}
                        box{<eF1+eF2,eF1+eF2,-1>,<xF-eF1-eF2,yF-eF1-eF2,zF*3>}
                        texture{txtF2}
                }       
        // Vertical bars
                #if (nFx>0)
                        #local Count=0;    
                        #local imF=(xF-2*(eF1+eF2))/(nFx+1);
                        #while (Count<nFx)
                                box{<0,eF2+eF1,0>,<eF3,yF-eF2-eF1,eF3>
                                translate <-eF3*0.5,0,-eF3*0.5>
                                rotate y*45
                                scale <2,1,1>
                                translate z*eF2*1.414
                                translate x*((Count+1)*imF+eF1+eF2)
                                texture{txtF2}
                                }
                        #local Count=Count+1;
                        #end
                #end
        // Horizontal bars
                #if (nFy>0)
                        #local Count=0;
                        #local imF=(yF-2*(eF1+eF2))/(nFy+1);
                        #while (Count<nFy)
                                box{<eF1+eF2,0,0>,<xF-eF1-eF2,eF3,eF3>
                                translate <0,-eF3*0.5,-eF3*0.5>
                                rotate x*45
                                scale <1,2,1>
                                translate z*eF2*1.414
                                translate y*((Count+1)*imF+eF1+eF2)
                                texture{txtF2}
                                }
                        #local Count=Count+1;
                        #end       
                #end
}        
//-----------------------------------------
// puts the window together
//-----------------------------------------
#if (typeF=1)
        object{Battant
                rotate -y*tetaFG
                translate -x*xF1
                texture{txtF1}
        }
#else
        union{
                object{Battant
                        rotate -y*tetaFG
                        translate -x*xF
                }
                union{
                        object{Battant}            
                        box{<xF-eF1+eF2,0,-eF2>,<xF+eF1-eF2,yF,0>}
                        cylinder{0,y*yF,(eF1-eF2)*0.8 scale <1,1,0.5> translate x*xF}
                        rotate -y*tetaFD
                        translate -x*xF
                        scale <-1,1,1>
                }
                texture{txtF1}
        }        
#end
#end