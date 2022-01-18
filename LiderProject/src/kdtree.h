/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}


	void insert_helper(Node **n ,int depth ,pcl::PointXYZI  point, int id)
	{
		if(*n ==NULL)
		   *n =new Node (point ,id);
		else{

			int cd = depth%3;
			if(cd ==0)
			   {
                if(point.x<((*n)->point.x))
			      insert_helper(&((*n)->left),depth+1,point,id);
			    else
			      insert_helper(&((*n)->right),depth+1,point,id);
			   }
			else if(cd ==1)
				{
                if(point.y<((*n)->point.y))
			      insert_helper(&((*n)->left),depth+1,point,id);
			    else
			      insert_helper(&((*n)->right),depth+1,point,id);
				}
			else
				{
                if(point.z<((*n)->point.z))
			      insert_helper(&((*n)->left),depth+1,point,id);
			    else
			      insert_helper(&((*n)->right),depth+1,point,id);
				}

		}
		  
		
	}
		void insert(pcl::PointXYZI  point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root,0,point,id);
		

	}
		void search_helper(Node *n ,int depth ,pcl::PointXYZI  target,float distanceTol, std::vector<int>& ids)
	{
		if(n!=NULL)
		{
		if(((n->point.x>=(target.x-distanceTol)) &&(n->point.x<=(target.x+distanceTol)))
		   &&((n->point.y>=(target.y-distanceTol)) &&(n->point.y<=(target.y+distanceTol)))
		   &&((n->point.z>=(target.z-distanceTol)) &&(n->point.z<=(target.z+distanceTol))))
		   {
			   float dis =sqrt((n->point.x-target.x)*(n->point.x-target.x)+(n->point.y-target.y)*(n->point.y-target.y)+(n->point.z-target.z)*(n->point.z-target.z));
			   if(dis<=distanceTol)
                 ids.push_back(n->id);
		   }
		
    
			int cd = depth%3;
				if(cd ==0)
			   {
			    if((target.x-distanceTol)<((n)->point.x))
			      search_helper(n->left,depth+1,target,distanceTol, ids);
			    if((target.x+distanceTol)>((n)->point.x))
			      search_helper(n->right,depth+1,target,distanceTol, ids);
			   }
				else if(cd ==1)
			   {
			    if((target.y-distanceTol)<((n)->point.y))
			      search_helper(n->left,depth+1,target,distanceTol, ids);
			    if((target.y+distanceTol)>((n)->point.y))
			      search_helper(n->right,depth+1,target,distanceTol, ids);
			   }
				else
			   {
			    if((target.z-distanceTol)<((n)->point.z))
			      search_helper(n->left,depth+1,target,distanceTol, ids);
			    if((target.z+distanceTol)>((n)->point.z))
			      search_helper(n->right,depth+1,target,distanceTol, ids);
			   }

		
		  
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root,0 ,target,distanceTol, ids);
		return ids;
	}
	

};




