#pragma once

template<class Ti>
class TQuadtree;

struct TQuadtreeNode
{
	int is_end;
	TAABB2 dims;
	int child[2][2]; //child[0-min,1-max][]
	TQuadtreeNode():is_end(-1){for(int i=0;i<4;i++)child[0][i]=-1;}
};

template<class Ti>
class TQuadtree
{
private:
	TIndexedPointerArray<TQuadtreeNode> nodes;
	float min_node_size;
	int root;
	TIndexedPointerArray<Ti> items;
	void Init(TQuadtreeNode* node,const TAABB2& use_dims)
	{
		node->dims=use_dims;
		if((node->dims.Get(0,1)-node->dims.Get(0,0))<min_node_size*2)
		{
			node->is_end=items.Newi();
		}
	}
public:
	TQuadtree(float use_min_node_size,const TAABB2& use_dims)
	{
		min_node_size=use_min_node_size;
		root=nodes.Newi();
		Init(nodes[root],use_dims);
	}
	template<class T>
	void Add(const TAABB2& use_dims,T& args)
	{
		if(use_dims.Overlaps(nodes[root]->dims))
			Add(nodes[root],use_dims,args);
	}

	//template<class T>
	//void Add(TQuadtreeNode* node,const TAABB2& use_dims,T& args)
	//{
	//	//TODO accept node если полностью в use_dims
	//	int i,k,t;
	//	if(node->is_end!=-1)items[node->is_end]->Add(use_dims,args);
	//	else
	//		for(i=0;i<2;i++)
	//			if((use_dims.Get(0,i)<(node->dims.Get(0,0)+node->dims.Get(0,1))*0.5)!=i)
	//				for(k=0;k<2;k++)
	//					if((use_dims.Get(1,k)<(node->dims.Get(1,0)+node->dims.Get(1,1))*0.5)!=k)
	//						for(t=0;t<2;t++)
	//							if((use_dims.Get(2,t)<(node->dims.Get(2,0)+node->dims.Get(2,1))*0.5)!=t)
	//							{
	//								if(node->child[i][k][t]==-1)
	//								{
	//									node->child[i][k][t]=nodes.Newi();
	//									TAABB temp(node->dims);
	//									temp.ToSubCube(i,k,t);
	//									Init(nodes[node->child[i][k][t]],temp);
	//								}
	//								Add(nodes[node->child[i][k][t]],use_dims,args);
	//							}
	//}

	template<class T>
	void Del(const TAABB2& use_dims,T& args)
	{
		if(use_dims.Overlaps(nodes[root]->dims))
			Del(nodes[root],use_dims,args);
	}

	
	//template<class T>
	//void Del(TQuadtreeNode* node,const TAABB2& use_dims,T& args)
	//{
	//	int i,k,t;
	//	if(node->is_end!=-1)
	//		items[node->is_end]->Del(use_dims,args);
	//	else
	//		for(i=0;i<2;i++)
	//			if((use_dims.Get(0,i)<(node->dims.Get(0,0)+node->dims.Get(0,1))*0.5)!=i)
	//				for(k=0;k<2;k++)
	//					if((use_dims.Get(1,k)<(node->dims.Get(1,0)+node->dims.Get(1,1))*0.5)!=k)
	//						Del(nodes[node->child[i][k][t]],use_dims,args);
	//}

	template<class T>
	void GetOverlaped(const TAABB2& use_dims,T& args)
	{
		GetOverlaped(nodes[root],use_dims,args);
	}		

	//template<class T>
	//void GetOverlaped(TQuadtreeNode* node,const TAABB2& use_dims,T& args)
	//{
	//	if(is_end.isNotNil())is_end->GetOverlaped(use_dims,args);
	//	else
	//		for(int i=0;i<2;i++)
	//			if(child[i].isNotNil()&&((use_dims.Get(div_border_type,i)<div_border)!=i))
	//				child[i]->GetOverlaped(use_dims,args);
	//}
};
