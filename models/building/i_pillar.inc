// *** randomly textured pillar macro ***
#macro pillar(t_pm,r_p)

union{
 cylinder{<0,0,0>,<0,0.5,0>,2}
 cylinder{<0,0.5,0>,<0,1.5,0>,1.5}
 cylinder{<0,1.5,0>,<0,10,0>,1.25}
 cylinder{<0,10,0>,<0,10.5,0>,1.5}
 sphere{0,1.85 translate 12.5*y}
 texture{t_pm translate 100*r_p}
}

#end
