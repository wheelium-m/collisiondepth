#version 110
#extension GL_EXT_gpu_shader4: enable

varying vec2 texcoord;
varying float zcoord;
uniform sampler2D texture;

void main(){
     
     
     vec4 floats=texture2D(texture, texcoord);
     float zdepth = -1.0*floats[0];
     if(zcoord>zdepth)
	gl_FragColor = vec4(0.0,1.0,0.0,0.0);
     else
	gl_FragColor = vec4(1.0,0.0,0.0,0.0);
     /*if(zdepth > -0.6 && zdepth < -0.4)
     	 gl_FragColor = vec4(0.0,1.0,0.0,0.0);
     else
	gl_FragColor = vec4(1.0,0.0,0.0,0.0);*/
}