// ### CIGARRETES IN THE STREET ###

// --- cigarretes on the tree terrain ---
#declare r_cig=seed(367);
#declare num_cig=12;
#declare cnt_cig=1;
#declare cig_on_tree=
union{
#while (cnt_cig<=num_cig)
 union{
  cylinder{<0,0,0>,<0,0,2.5>,.8
   texture{pigment{Orange}}
  }
  cylinder{<0,0,2.5>,<0,0,3+2*rand(r_cig)>,.8
   texture{pigment{White}}
  }
  rotate 360*rand(r_cig)*y
  translate <-65+130*rand(r_cig),-1,-65+130*rand(r_cig)>
 }
 #declare cnt_cig=cnt_cig+1;
#end
}

// --- cigarretes on the sidewalk ---
#declare num_cig=10;
#declare cnt_cig=1;
#declare cig_on_sidew=
union{
#while (cnt_cig<=num_cig)
 union{
  cylinder{<0,0,0>,<0,0,2.5>,.8
   texture{pigment{Orange}}
  }
  cylinder{<0,0,2.5>,<0,0,3+2*rand(r_cig)>,.8
   texture{pigment{White}}
  }
  rotate 360*rand(r_cig)*y
  translate <-300+600*rand(r_cig),15,300+295*rand(r_cig)>
 }
 #declare cnt_cig=cnt_cig+1;
#end
}

// --- cigarretes on street ---
#declare num_cig=28;
#declare cnt_cig=1;
#declare cig_on_street=
union{
#while (cnt_cig<=num_cig)
 union{
  cylinder{<0,0,0>,<0,0,2.5>,.8
   texture{pigment{Orange}}
  }
  cylinder{<0,0,2.5>,<0,0,3+2*rand(r_cig)>,.8
   texture{pigment{White}}
  }
  rotate 360*rand(r_cig)*y
  translate <-300+600*rand(r_cig),3.5,190+105*rand(r_cig)>
 }
 #declare cnt_cig=cnt_cig+1;
#end
}
