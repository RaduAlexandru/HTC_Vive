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

class Animation:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */

	public:
	private:


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

/*
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
}*/

Animation::Animation(int& argc,char**& argv)
	:Vrui::Application(argc,argv)
	{


	//makeCheckImage();

	
	Vertex v=  { -0.5, -0.5, 0.0, 0.0, 0.0};
	vertices.push_back(v);
	Vertex v2=  {0.5, 0.0, 0.0, 1.0, 0.0};
	vertices.push_back(v2);
	Vertex v3=  {0.0, 0.5, 0.0, 0.0, 1.0};
	vertices.push_back(v3);
	
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

	
	/*glPushMatrix();
	
	glTranslated(-5.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.5f,0.5f));
	glDrawCube(7.5f);
	
	glTranslated(10.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,1.0f));
	glDrawSphereIcosahedron(4.5f,6);
	
	glPopMatrix();*/

	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);

	// Black background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw VBO
	glPushMatrix();
	glBindBufferARB(GL_ARRAY_BUFFER, dataItem->VBO_id[0]);
	glVertexPointer(3, GL_FLOAT, sizeof(Vertex), 0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	glPopMatrix();


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


	/*Vertex vertices[] = {
	{-0.5, -0.5, 0.0, 0.0, 0.0},
	{0.5, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.5, 0.0, 0.0, 0.0},
	};*/
	



//	glBindBufferARB(GL_ARRAY_BUFFER,dataItem-> VBO_id[0]);
//	glBufferDataARB(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);	



	/* Upload all vertices into the vertex buffer: */
  	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[0]);
  	glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices.size()*sizeof(Vertex),&vertices[0],GL_STATIC_DRAW_ARB);
  	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);

  	/* Upload all vertex indices into the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[0]);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices.size()*sizeof(GLuint),&indices[0],GL_STATIC_DRAW_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);



	}

VRUI_APPLICATION_RUN(Animation)
