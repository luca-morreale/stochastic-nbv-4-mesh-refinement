/* --- randomize scale and translate ---
   description:
    useful to randomize textures
   parameters :
    + sc -> scaling range
    + tc -> translating range
    + r  -> random seed
   ------------------------------------- */
#macro Rnd_scatrans(sc,tc,rn)

 scale 1+sc*rand(rn)
 translate tc*rand(rn)

#end



/*
  *** HANGING CABLE MACRO ***

  parameters:

    start point
    end point
    cable diamenter
    number of sections
    curve radius
    cable texture

  ***************************
*/

#macro hanging_cable(p1,p2,d,sc,r,tx)

 #local cnt=1;
 #local x_ant=p1.x;
 #local y_ant=p1.y;
 #local z_ant=p1.z;
 #local x_inc=abs(p2.x-p1.x)/sc;
 #local z_inc=abs(p2.z-p1.z)/sc;

 union{
  #while (cnt<=sc)
    #local y_inc=r*abs(sin(3.141516*cnt/sc));
    #if (p2.x>p1.x)
     #local x_act=x_ant+x_inc;
    #else
     #local x_act=x_ant-x_inc;
    #end
    #if (p2.y>p1.y)
     #local y_act=p1.y-y_inc+(cnt*abs(p2.y-p1.y)/sc);
    #else
     #local y_act=p1.y-y_inc-(cnt*abs(p2.y-p1.y)/sc);
    #end
    #if (p2.z>p1.z)
     #local z_act=z_ant+z_inc;
    #else
     #local z_act=z_ant-z_inc;
    #end
    cylinder{
     <x_ant,y_ant,z_ant>
     <x_act,y_act,z_act>
     d*.5
    }
    #declare x_ant=x_act;
    #declare y_ant=y_act;
    #declare z_ant=z_act;
    #declare cnt=cnt+1;
  #end
  texture{tx}
 }

#end




// *** random wrinkled paper macro ***
//
// params: random seed, scale

#macro wrinkled_paper(rseed,sc)

#local rp=rseed;
#local s1=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s2=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s3=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s4=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s5=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s6=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s7=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s8=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s9=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s10=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s11=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s12=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s13=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s14=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s15=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;
#local s16=<-1+2*rand(rp),-.5+1*rand(rp),-1+2*rand(rp)>;

bicubic_patch {
   type 1 flatness 0.01 u_steps 4 v_steps 4,
   s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16
   scale sc
   rotate rand(rp)*360*y
}

#end