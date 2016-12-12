/***********************************************************************
Animation - Example program demonstrating data exchange between a
background animation thread and the foreground rendering thread using
a triple buffer, and retained-mode OpenGL rendering using vertex and
index buffers.
Copyright (c) 2014-2015 Oliver Kreylos

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2 of the License, or (at your
option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include <unistd.h>
#include <iostream>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/GLMaterial.h>
#include <GL/GLVertexTemplates.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLGeometryVertex.h>
#include <GL/GLVertex.h>
#include <GL/GLModels.h>
#include <GL/GLMaterialTemplates.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#define checkImageWidth 64
#define checkImageHeight 64

#include <map>

class Animation:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */


	struct Vertex
{
	float x;
	float y;
	float z;
	float u;
	float v;
};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint VBO_id[2];	
		GLuint indexBufferId[2];
		GLuint texture[2];
		bool rendering_buffer_0=true;

		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	private:
	std::vector <Vertex> vertices;
	std::vector<GLuint> indices;


	GLubyte checkImage[checkImageHeight][checkImageWidth][4];
	unsigned int version; // Version number of mesh in the most-recently locked triple buffer slot
	Threads::Thread read_thread;
	
	void* read_data(void);
	void makeCheckImage(void);
	void saveGLState(void);
	void restoreGLState();
	void SetGLCapability(GLenum capability, GLboolean state);

	GLboolean SavedLighting;
    	GLboolean SavedDepthTest; 
    	GLboolean SavedBlending;
	std::map<std::string, int> GLStateIntegers;	

	
	/* Constructors and destructors: */
	public:
	Animation(int& argc,char**& argv);
	virtual ~Animation(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;


	};

/***********************************
Methods of class Anmation::DataItem:
***********************************/

Animation::DataItem::DataItem(void)
	:rendering_buffer_0(true)
	{
	/* Initialize the GL_ARB_vertex_buffer_object extension: */
	GLARBVertexBufferObject::initExtension();

	glGenBuffersARB(2, VBO_id);
	glGenBuffersARB(2, indexBufferId);
	glGenBuffersARB(2, texture);
		
	}

Animation::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */


	//glDeleteBuffersARB(1,&vertexBufferObjectId_0);
	//glDeleteBuffersARB(1,&vertexBufferObjectId_1);
	//glDeleteBuffersARB(1,&elementbuffer_0);
	//glDeleteBuffersARB(1,&elementbuffer_1);
	}

/**************************
Methods of class Animation:
**************************/

void* Animation::read_data(void){




}


void Animation::makeCheckImage(void)
{
   int i, j, c;

   for (i = 0; i < checkImageHeight; i++) {
      for (j = 0; j < checkImageWidth; j++) {
         c = ((((i&0x8)==0)^((j&0x8))==0))*255;
         checkImage[i][j][0] = (GLubyte) c;
         checkImage[i][j][1] = (GLubyte) c;
         checkImage[i][j][2] = (GLubyte) c;
         checkImage[i][j][3] = (GLubyte) 255;
      }
   }
}

Animation::Animation(int& argc,char**& argv)
	:Vrui::Application(argc,argv)
	{


	makeCheckImage();

	
	Vertex v2=  {5, 0.0, 0.0, 1.0, 0.0};
	vertices.push_back(v2);
	Vertex v=  { -5, -5, 0.0, 0.0, 0.0};
	vertices.push_back(v);
	Vertex v3=  {0.0, 5, 0.0, 0.0, 1.0};
	vertices.push_back(v3);


	indices.push_back(0);
	indices.push_back(1);
	indices.push_back(2);
	
	read_thread.start(this,&Animation::read_data);
	}

Animation::~Animation(void)
	{
	/* Shut down the background animation thread: */
	read_thread.cancel();
	read_thread.join();
	
}

void Animation::frame(void)
	{
	/* Check if there is a new entry in the triple buffer and lock it: */
	//if(meshVertices.lockNewValue())
	//	{
		/* Invalidate the in-GPU vertex buffer: */
	//	++version;
	//	}
	}

void Animation::display(GLContextData& contextData) const
	{

	
/*	glPushMatrix();
	
	glTranslated(-5.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.5f,0.5f));
	glDrawCube(7.5f);
	
	glTranslated(10.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,1.0f));
	glDrawSphereIcosahedron(4.5f,6);
	
	glPopMatrix();
*/
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);

	// Black background
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




	glEnableClientState(GL_VERTEX_ARRAY);
 	glEnableClientState(GL_TEXTURE_COORD_ARRAY);


	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[0]);
 	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[0]);
  	glBindTexture(GL_TEXTURE_2D, dataItem->texture[0]);  //TODO maybe  I also need to bund the texture

  	glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*5, NULL);
  	glTexCoordPointer(2, GL_FLOAT, sizeof(GLfloat)*5, (float*)(sizeof(GLfloat)*3)); //TODO this may need to change to a 2




	 glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping ( NEW )
    	glShadeModel(GL_SMOOTH);                        // Enable Smooth Shading
    	//glClearColor(0.0f, 0.0f, 0.0f, 0.5f);                   // Black Background
    	//glClearDepth(1.0f);                         // Depth Buffer Setup
    	//glEnable(GL_DEPTH_TEST);                        // Enables Depth Testing
    	//glDepthFunc(GL_LEQUAL);     





	//save stuff
	const_cast<Animation*>( this )->        saveGLState();
        glPushAttrib( GL_ALL_ATTRIB_BITS );
        glPushClientAttrib( GL_CLIENT_ALL_ATTRIB_BITS );
        glMatrixMode( GL_COLOR );
        glPushMatrix();
        glMatrixMode( GL_TEXTURE );
        glPushMatrix();
        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();

	//glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, indexPtr);
	//glDrawArrays(GL_TRIANGLES, 0, 3);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, NULL);
	//restore stuff

	glMatrixMode( GL_COLOR );
        glPopMatrix();
        glMatrixMode( GL_TEXTURE );
        glPopMatrix();
        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPopMatrix();
        glPopAttrib();
        glPopClientAttrib();
        const_cast<Animation*>( this )->        restoreGLState();


	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
  	glBindTexture(GL_TEXTURE_2D,0);

  	glDisableClientState(GL_VERTEX_ARRAY);
  	glDisableClientState(GL_TEXTURE_COORD_ARRAY);




	/*// Draw VBO
	const_cast<Animation*>( this )->	saveGLState();



	glPushAttrib( GL_ALL_ATTRIB_BITS );
        glPushClientAttrib( GL_CLIENT_ALL_ATTRIB_BITS );
        glMatrixMode( GL_COLOR );
        glPushMatrix();
        glMatrixMode( GL_TEXTURE );
        glPushMatrix();
        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();

	//glPushMatrix();
	glBindBufferARB(GL_ARRAY_BUFFER, dataItem->VBO_id[0]);
	glVertexPointer(3, GL_FLOAT, sizeof(Vertex), 0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	//glPopMatrix();


	
        glMatrixMode( GL_COLOR );
        glPopMatrix();
        glMatrixMode( GL_TEXTURE );
        glPopMatrix();
        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPopMatrix();
        glPopAttrib();
        glPopClientAttrib();


	const_cast<Animation*>( this )->	restoreGLState();*/

}


void Animation::saveGLState(void){

   glGetIntegerv(GL_ACTIVE_TEXTURE, &this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);




	int max;
	GLint value;
    glGetIntegerv(static_cast<GLenum>(0x8B4D),&value);
    max=static_cast<int>(value);


//	std::cout << "max units is" << max<< std::endl;
if (this->GLStateIntegers["GL_ACTIVE_TEXTURE"] < 0 || this->GLStateIntegers["GL_ACTIVE_TEXTURE"] > max ){

	this->GLStateIntegers["GL_ACTIVE_TEXTURE"] = 0;
}	
	

    this->SavedLighting = glIsEnabled(GL_LIGHTING);
    this->SavedDepthTest = glIsEnabled(GL_DEPTH_TEST);
    this->SavedBlending = glIsEnabled(GL_BLEND);


}

void Animation::restoreGLState(void){

	SetGLCapability(GL_LIGHTING, SavedLighting);
        SetGLCapability(GL_DEPTH_TEST, SavedDepthTest);
        SetGLCapability(GL_BLEND, SavedBlending);


	glActiveTexture(GL_TEXTURE0 + this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);

}


void Animation::SetGLCapability(GLenum capability, GLboolean state){
    if (state)
    {
      glEnable(capability);
    }
    else
    {
      glDisable(capability);
    }
}

void Animation::resetNavigation(void)
	{
	/* Center and scale the object: */
	//	Vrui::setNavigationTransformation(Vrui::Point::origin,9.0*Math::Constants<double>::pi,Vrui::Vector(0,1,0));
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12),Vrui::Vector(0,1,0));
	}

void Animation::initContext(GLContextData& contextData) const
	{



	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);




	/* Upload all vertices into the vertex buffer: */
  	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[0]);
  	glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices.size()*sizeof(Vertex),&vertices[0],GL_STATIC_DRAW_ARB);
  	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);

  	/* Upload all vertex indices into the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[0]);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices.size()*sizeof(GLuint),&indices[0],GL_STATIC_DRAW_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);




	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, dataItem->texture[0]);

   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth, 
                checkImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                checkImage);
	}



	//	GLuint shaderProgram = create_program("shaders/vert.shader", "shaders/frag.shader");

VRUI_APPLICATION_RUN(Animation)
