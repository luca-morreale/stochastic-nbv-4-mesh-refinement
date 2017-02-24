// *** randomly textured pillar macro ***
#macro pillar(t_pm,r_p)

union{
 cylinder{<0,0,0>,<0,2,0>,8}
 cylinder{<0,2,0>,<0,6,0>,6}
 cylinder{<0,6,0>,<0,40,0>,5}
 cylinder{<0,40,0>,<0,42,0>,6}
 sphere{0,5.5 translate 47*y}
 texture{t_pm translate 100*r_p}
}

#end