#pragma once

template<class Ti>
class TOctree;

struct TOctreeNode
{
	TOctreeNode():is_end(-1){for(int i=0;i<8;i++)child[0][0][i]=-1;}
	int is_end;
	TAABB dims;
	int child[2][2][2]; //child[0-min,1-max][][]
};

template<class Ti>
class TOctree
{
private:
	TIndexedPointerArray<TOctreeNode> nodes;
	float min_node_size;
	int root;
	TIndexedPointerArray<Ti> items;
public:
	TOctree(float use_min_node_size,const TAABB& use_dims)
	{
		min_node_size=use_min_node_size;
		root=nodes.Newi();
		Init(nodes[root],use_dims);
	}

	void Init(TOctreeNode* node,const TAABB& use_dims)
	{
		node->dims=use_dims;
		if((node->dims.Get(0,1)-node->dims.Get(0,0))<min_node_size*2)
		{
			node->is_end=items.Newi();
		}
	}

	template<class T>
	void Add(const TAABB& use_dims,T& args)
	{
		if(use_dims.Overlaps(nodes[root]->dims))
			Add(nodes[root],use_dims,args);
	}

	template<class T>
	void Add(TOctreeNode* node,const TAABB& use_dims,T& args)
	{
		int i,k,t;
		if(node->is_end!=-1)items[node->is_end]->Add(use_dims,args);
		else
			for(i=0;i<2;i++)
				if((use_dims.Get(0,i)<(node->dims.Get(0,0)+node->dims.Get(0,1))*0.5)!=i)
					for(k=0;k<2;k++)
						if((use_dims.Get(1,k)<(node->dims.Get(1,0)+node->dims.Get(1,1))*0.5)!=k)
							for(t=0;t<2;t++)
								if((use_dims.Get(2,t)<(node->dims.Get(2,0)+node->dims.Get(2,1))*0.5)!=t)
								{
									if(node->child[i][k][t]==-1)
									{
										node->child[i][k][t]=nodes.Newi();
										TAABB temp(node->dims);
										temp.ToSubCube(i,k,t);
										Init(nodes[node->child[i][k][t]],temp);
									}
									Add(nodes[node->child[i][k][t]],use_dims,args);
								}
	}

	template<class T>
	void Del(const TAABB& use_dims,T& args)
	{
		if(use_dims.Overlaps(nodes[root]->dims))
			Del(nodes[root],use_dims,args);
	}

	
	template<class T>
	void Del(TOctreeNode* node,const TAABB& use_dims,T& args)
	{
		int i,k,t;
		if(node->is_end!=-1)
			items[node->is_end]->Del(use_dims,args);
		else
			for(i=0;i<2;i++)
				if((use_dims.Get(0,i)<(node->dims.Get(0,0)+node->dims.Get(0,1))*0.5)!=i)
					for(k=0;k<2;k++)
						if((use_dims.Get(1,k)<(node->dims.Get(1,0)+node->dims.Get(1,1))*0.5)!=k)
							for(t=0;t<2;t++)
								if((use_dims.Get(2,t)<(node->dims.Get(2,0)+node->dims.Get(2,1))*0.5)!=t)
									Del(nodes[node->child[i][k][t]],use_dims,args);
	}

	void Draw()
	{
		Draw(nodes[root]);
	}
	void Draw(TOctreeNode* node)
	{
		if(node->is_end!=-1)
			node->dims.DrawLine();
		else
		for(int i=0;i<8;i++)
			if(node->child[0][0][i]!=-1)
				Draw(nodes[node->child[0][0][i]]);
	}

	template<class T>
	void inFrustrum(const TFrustrum& frustrum,T& args)
	{
		InFrustrum(nodes[root],frustrum,args);
	}

	template<class T>
	void PointIn(const TVec3& point,T& args)
	{
		PointIn(nodes[root],point,args);
	}

	template<class T>
	void GetOverlaped(const TAABB& use_dims,T& args)
	{
		GetOverlaped(nodes[root],use_dims,args);
	}		

	template<class T>
	void InFrustrum(TOctreeNode* node,const TFrustrum& frustrum,T& args)
	{
		int i,k,t;
		if(node->is_end!=-1)items[node->is_end]->InFrustrum(frustrum,args);
		else
			for(i=0;i<2;i++)
				for(k=0;k<2;k++)
					for(t=0;t<2;t++)
						if(node->child[i][k][t]!=-1)
						{
							switch(frustrum.Overlapsi(nodes[node->child[i][k][t]]->dims))
							{
							case 1:
								InFrustrum(nodes[node->child[i][k][t]],frustrum,args);
								break;
							case 2:
								InFrustrumAdd(nodes[node->child[i][k][t]],frustrum,args);
							}
						}
	}

	template<class T>
	void InFrustrumAdd(TOctreeNode* node,const TFrustrum& frustrum,T& args)
	{
		int i;
		if(node->is_end!=-1)items[node->is_end]->InFrustrumAdd(frustrum,args);
		else
			for(i=0;i<8;i++)
				if(node->child[0][0][i]!=-1)
					InFrustrumAdd(nodes[node->child[0][0][i]],frustrum,args);
	}

	//template<class T>
	//void PointIn(TOctreeNode* node,const TVec3& point,T& args)
	//{
	//	//TODO
	//	if(node->is_end!=-1)is_end->PointIn(point,args);
	//	else
	//	{
	//		bool t=point[div_border_type]<div_border;
	//		if(child[t].isNotNil())
	//			child[t]->PointIn(point,args);
	//	}
	//}

	//template<class T>
	//void GetOverlaped(TOctreeNode* node,const TAABB& use_dims,T& args)
	//{
	//	//TODO
	//	if(node->is_end!=-1)is_end->GetOverlaped(use_dims,args);
	//	else
	//		for(int i=0;i<2;i++)
	//			if(child[i].isNotNil()&&((use_dims.Get(div_border_type,i)<div_border)!=i))
	//				child[i]->GetOverlaped(use_dims,args);
	//}
};

