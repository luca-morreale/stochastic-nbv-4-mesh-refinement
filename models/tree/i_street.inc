// ### STREE COMPONENTS ###
// *** street coblestones ***
#declare cobble=
height_field{
 png "hf_cobble"
 texture{
  t_cobble_map
  rotate 90*x
 }
 translate -.5
 scale <300,15,600>
}
// *** street cobb. with hole for sewer ***
#declare cobbleh=
difference{
 height_field{
  png "hf_cobble"
  texture{
   t_cobble_map
   rotate 90*x
  }
  translate -.5
  scale <300,15,600>
 }
 cylinder{
  <100,0,300>
  <150,0,300>
  40
  scale <1,.5,1>
  texture{T_Stone3}
 }
}
// ### SIDEWALK ###
// *** normal sidewalk ***
#declare sidew=
height_field{
 png "hf_sidew"
 texture{
  t_sidew
  rotate 90*x
 }
 translate -.5
 scale <300,15,300>
}
// *** sidewalk with holes for tree and sewer tap ***
#declare sidew2=
height_field{
 png "hf_sidew2"
 texture{
  t_sidew
  rotate 90*x
 }
 translate -.5
 scale <300,15,300>
}
// *** sidewalk border front ***
#declare sidewb=
height_field{
 png "hf_sidewb"
 texture{
  t_sidewb
  rotate 90*x
 }
 translate -.5
 rotate -90*x
 scale <300,15,15>
}
// *** sidewalk border top ***
#declare sidewb2=
height_field{
 png "hf_sidewb"
 texture{
  t_sidewb
  rotate 90*x
 }
 translate -.5
 scale <300,15,20>
}
// *** sidewalk border front with hole for sewer ***
#declare sidewbh=
height_field{
 png "hf_sidewbh"
 texture{
  t_sidewb
  rotate 90*x
 }
 translate -.5
 rotate -90*x
 scale <300,15,15>
}
// *** sidewalk border top with hole for sewer ***
#declare sidewb2h=
height_field{
 png "hf_sidewbh"
 texture{
  t_sidewb
  rotate 90*x
 }
 translate -.5
 scale <300,15,20>
}
// *** tree border stones ***
#declare sidewb3=
height_field{
 png "hf_sidewb"
 texture{
  t_sidew
  rotate 90*x
 }
 translate -.5
 scale <150,15,10>
}
// *** tree terrain ***
#declare terrain=
height_field{
 png "hf_terr"
 texture{t_terrain}
 translate -.5
 scale <150,20,150>
}
// *** sewer tap ***
#declare tap=
box{
 <-25,0,-25>,<25,1,25>
 texture{t_tap_metal}
}
// *** false rear street, only for reflections ***
#if (trace_at_night)
#declare rear_street=
box{
 <-1600,0,-10>
 <1600,500,0>
 texture{
  pigment{image_map{png "im_alley"} translate -.5}
  finish{Luminous ambient 2}
  scale <400,300,1>
  translate 150*y
 }
 translate -50*x
}
#else
#declare rear_street=
box{
 <-1600,0,-10>
 <1600,500,0>
 texture{
  Mirror
 }
 translate -50*x
}
#end
// *** sewer entry **
difference{
 box{<-25,0,-5>,< 26,15,15>}
 box{<-22,0,-6>,<-19,14,14>}
 box{<-17,0,-6>,<-14,14,14>}
 box{<-12,0,-6>,< -9,14,14>}
 box{< -7,0,-6>,< -4,14,14>}
 box{< -2,0,-6>,<  1,14,14>}
 box{<  3,0,-6>,<  6,14,14>}
 box{<  8,0,-6>,< 11,14,14>}
 box{< 13,0,-6>,< 16,14,14>}
 box{< 18,0,-6>,< 21,14,14>}
 texture{t_tap_metal translate 40}
 translate <-175,0,283>
}
