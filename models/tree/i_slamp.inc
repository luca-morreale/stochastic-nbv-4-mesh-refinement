// *** clasical old city lamp ***
#declare glass_sheet=
prism{
 linear_sweep
 linear_spline
 0,
 0.2,
 5,
 <-4.5,0> <4.5,0> <6.5,16> <-6.5,16> <-4.5,0>
 hollow
 rotate -96*x
}
#declare top=
intersection{
 superellipsoid{
  <.1,1>
  rotate 90*x
  scale <6.5,3,6.5>
 }
 plane{y,0 inverse}
}
#declare candil=
union{
 union{
  box{<-7,0,-7>,<7,0.5,7> translate 16*y}
  difference{
   box{<-5,0,-5>,<5,0.5,5>}
   box{<-4.5,-.1,-4.5>,<4.5,0.6,4.5>}
  }
  cylinder{<-4.5,.5,-4.5>,<-6.5,16,-6.5>,.3}
  cylinder{< 4.5,.5,-4.5>,< 6.5,16,-6.5>,.3}
  cylinder{<-4.5,.5, 4.5>,<-6.5,16, 6.5>,.3}
  cylinder{< 4.5,.5, 4.5>,< 6.5,16, 6.5>,.3}
  cylinder{<0,0.5,0>,<0,1,0>,4}
  cylinder{<0,1,0>,<0,2,0>,3}
  cylinder{<0,2,0>,<0,5,0>,2}
  cylinder{<0,5,0>,<0,8,0>,1}
  object{top translate 16.5*y}
  sphere{<0,0,0>,2 translate 20*y}
  sphere{<0,0,0>,3 scale <1,0.5,1> translate 19*y}
  object{top scale 0.8 rotate 180*x translate .5*y}
  sphere{<0,0,0>,2 translate -2*y}
  texture{t_pillar_metal}
 }
 sphere{
  0,3 scale <1,1.5,1>
  texture{Glass2}
  translate 10*y
 }
 union{
  object{glass_sheet translate <0,0.5,-4.5>}
  object{glass_sheet translate <0,0.5,-4.5> rotate 90*y}
  object{glass_sheet translate <0,0.5,-4.5> rotate 90*2*y}
  object{glass_sheet translate <0,0.5,-4.5> rotate 90*3*y}
  texture{
   pigment{White filter 0.2}
   #if (trace_at_night)
    finish{ambient (White*.8+Gold*.3)*8}
   #else
    finish{Glass_Finish}
   #end
  }
  no_shadow
 }
 #if (trace_at_night)
 sphere{<0,0,0>,1
   pigment { color rgbf<1, 1, 1, 1> }
   finish { ambient 0 diffuse 0 }
   interior{
     media {
      emission 0.05
      intervals 10
      samples 1, 10
      confidence 0.9999
      variance 1/1000
      density {
        spherical
        cubic_wave
        turbulence .1
        color_map {
          [0.0 Black]
          [0.5 Gold*.2]
          [1.0 Gold+White*.5]
        }
        frequency 1
      }
     }
   }
   no_shadow
   hollow
   scale <1,1,1>
   scale 18
   translate 10*y
 }
 light_source{
  0
  #if (trace_area)
   Gold*.8+White*.6
   area_light
   <6,0,0> <0,0,6>
   4, 4
   adaptive 1
   jitter
  #else
   Gold*.7+White*.5
  #end
  fade_distance 375
  fade_power 2
  translate 10*y
 }
 light_source{
  <0,-300,-100>
  (Gold*.7+White*.5)*.25
  fade_distance 200
  fade_power 4
  shadowless
 }
 #end
}
#declare candil2=
#if (trace_at_night)
 light_source{
  0
  #if (trace_area)
   Gold*.75+White*.55
   area_light
   <5,0,0> <0,0,5>
   3, 3
   adaptive 1
   jitter
  #else
   Gold*.65+White*.45
  #end
  fade_distance 375
  fade_power 2
  translate 10*y
 }
#end

// *** complete lamp ***
#declare slamp=
union{
 object{candil translate <0,-1,-10>}
 union{
  height_field{
   png "hf_lmp_s"
   water_level 0.2
   translate -.5
   rotate -90*x
   scale <20,12,2>
   rotate -90*y
  }
  box{<-2,-6,9>,<2,6,10>}
  translate <0,-10,0>
  texture{t_pillar_metal}
 }
 scale 2
}
#declare slamp_2=
object{candil2 translate <0,-1,-10>}

