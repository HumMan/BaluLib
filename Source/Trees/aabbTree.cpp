/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "aabbTree.h"
using namespace BaluLib;

const int b2_nullNode = -1;
const float b2_aabbExtension = 0.1f;
const float b2_aabbMultiplier = 2.0f;
const float b2_maxFloat = FLT_MAX;

void* b2Alloc(int size)
{
	return malloc(size);
}

void b2Free(void* mem)
{
	free(mem);
}

b2DynamicTree::b2DynamicTree()
{
	m_root = b2_nullNode;

	m_nodeCapacity = 16;
	m_nodeCount = 0;
	m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
	memset(m_nodes, 0, m_nodeCapacity * sizeof(b2TreeNode));

	// Build a linked list for the free list.
	for (int i = 0; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = -1;
	}
	m_nodes[m_nodeCapacity - 1].next = b2_nullNode;
	m_nodes[m_nodeCapacity - 1].height = -1;
	m_freeList = 0;

	m_path = 0;

	m_insertionCount = 0;
}

b2DynamicTree::~b2DynamicTree()
{
	// This frees the entire tree in one shot.
	b2Free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
int b2DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == b2_nullNode)
	{
		assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = m_nodes;
		m_nodeCapacity *= 2;
		m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
		memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(b2TreeNode));
		b2Free(oldNodes);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity - 1].next = b2_nullNode;
		m_nodes[m_nodeCapacity - 1].height = -1;
		m_freeList = m_nodeCount;
	}

	// Peel a node off the free list.
	int nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = b2_nullNode;
	m_nodes[nodeId].child1 = b2_nullNode;
	m_nodes[nodeId].child2 = b2_nullNode;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = nullptr;
	++m_nodeCount;
	return nodeId;
}

// Return a node to the pool.
void b2DynamicTree::FreeNode(int nodeId)
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	assert(0 < m_nodeCount);
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = -1;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
int b2DynamicTree::CreateProxy(const TAABB2& aabb, void* userData)
{
	int proxyId = AllocateNode();

	// Fatten the aabb.
	TVec2 r(b2_aabbExtension, b2_aabbExtension);
	m_nodes[proxyId].aabb.GetLowerBound() = aabb.GetLowerBound() - r;
	m_nodes[proxyId].aabb.GetUpperBound() = aabb.GetUpperBound() + r;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	InsertLeaf(proxyId);

	return proxyId;
}

void b2DynamicTree::DestroyProxy(int proxyId)
{
	assert(0 <= proxyId && proxyId < m_nodeCapacity);
	assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

bool b2DynamicTree::MoveProxy(int proxyId, const TAABB2& aabb, const TVec2& displacement)
{
	assert(0 <= proxyId && proxyId < m_nodeCapacity);

	assert(m_nodes[proxyId].IsLeaf());

	if (m_nodes[proxyId].aabb.Contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	// Extend AABB.
	TAABB2 b = aabb;
	TVec2 r(b2_aabbExtension, b2_aabbExtension);
	b.GetLowerBound() = b.GetLowerBound() - r;
	b.GetUpperBound() = b.GetUpperBound() + r;

	// Predict AABB displacement.
	TVec2 d = displacement * b2_aabbMultiplier;

	if (d[0] < 0.0f)
	{
		b.GetLowerBound()[0] += d[0];
	}
	else
	{
		b.GetUpperBound()[0] += d[0];
	}

	if (d[1] < 0.0f)
	{
		b.GetLowerBound()[1] += d[1];
	}
	else
	{
		b.GetUpperBound()[1] += d[1];
	}

	m_nodes[proxyId].aabb = b;

	InsertLeaf(proxyId);
	return true;
}

void b2DynamicTree::InsertLeaf(int leaf)
{
	++m_insertionCount;

	if (m_root == b2_nullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = b2_nullNode;
		return;
	}

	// Find the best sibling for this node
	TAABB2 leafAABB = m_nodes[leaf].aabb;
	int index = m_root;
	while (m_nodes[index].IsLeaf() == false)
	{
		int child1 = m_nodes[index].child1;
		int child2 = m_nodes[index].child2;

		float area = m_nodes[index].aabb.GetPerimeter();

		TAABB2 combinedAABB;
		combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
		float combinedArea = combinedAABB.GetPerimeter();

		// Cost of creating a new parent for this node and the new leaf
		float cost = 2.0f * combinedArea;

		// Minimum cost of pushing the leaf further down the tree
		float inheritanceCost = 2.0f * (combinedArea - area);

		// Cost of descending into child1
		float cost1;
		if (m_nodes[child1].IsLeaf())
		{
			TAABB2 aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			cost1 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			TAABB2 aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			float oldArea = m_nodes[child1].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();
			cost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending into child2
		float cost2;
		if (m_nodes[child2].IsLeaf())
		{
			TAABB2 aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			cost2 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			TAABB2 aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			float oldArea = m_nodes[child2].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();
			cost2 = newArea - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if (cost < cost1 && cost < cost2)
		{
			break;
		}

		// Descend
		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

	int sibling = index;

	// Create a new parent.
	int oldParent = m_nodes[sibling].parent;
	int newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != b2_nullNode)
	{
		// The sibling was not the root.
		if (m_nodes[oldParent].child1 == sibling)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = m_nodes[leaf].parent;
	while (index != b2_nullNode)
	{
		index = Balance(index);

		int child1 = m_nodes[index].child1;
		int child2 = m_nodes[index].child2;

		assert(child1 != b2_nullNode);
		assert(child2 != b2_nullNode);

		m_nodes[index].height = 1 + Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

		index = m_nodes[index].parent;
	}

	//Validate();
}

void b2DynamicTree::RemoveLeaf(int leaf)
{
	if (leaf == m_root)
	{
		m_root = b2_nullNode;
		return;
	}

	int parent = m_nodes[leaf].parent;
	int grandParent = m_nodes[parent].parent;
	int sibling;
	if (m_nodes[parent].child1 == leaf)
	{
		sibling = m_nodes[parent].child2;
	}
	else
	{
		sibling = m_nodes[parent].child1;
	}

	if (grandParent != b2_nullNode)
	{
		// Destroy parent and connect sibling to grandParent.
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else
		{
			m_nodes[grandParent].child2 = sibling;
		}
		m_nodes[sibling].parent = grandParent;
		FreeNode(parent);

		// Adjust ancestor bounds.
		int index = grandParent;
		while (index != b2_nullNode)
		{
			index = Balance(index);

			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
			m_nodes[index].height = 1 + Max(m_nodes[child1].height, m_nodes[child2].height);

			index = m_nodes[index].parent;
		}
	}
	else
	{
		m_root = sibling;
		m_nodes[sibling].parent = b2_nullNode;
		FreeNode(parent);
	}

	//Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
int b2DynamicTree::Balance(int iA)
{
	assert(iA != b2_nullNode);

	b2TreeNode* A = m_nodes + iA;
	if (A->IsLeaf() || A->height < 2)
	{
		return iA;
	}

	int iB = A->child1;
	int iC = A->child2;
	assert(0 <= iB && iB < m_nodeCapacity);
	assert(0 <= iC && iC < m_nodeCapacity);

	b2TreeNode* B = m_nodes + iB;
	b2TreeNode* C = m_nodes + iC;

	int balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int iF = C->child1;
		int iG = C->child2;
		b2TreeNode* F = m_nodes + iF;
		b2TreeNode* G = m_nodes + iG;
		assert(0 <= iF && iF < m_nodeCapacity);
		assert(0 <= iG && iG < m_nodeCapacity);

		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != b2_nullNode)
		{
			if (m_nodes[C->parent].child1 == iA)
			{
				m_nodes[C->parent].child1 = iC;
			}
			else
			{
				assert(m_nodes[C->parent].child2 == iA);
				m_nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			m_root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb.Combine(B->aabb, G->aabb);
			C->aabb.Combine(A->aabb, F->aabb);

			A->height = 1 + Max(B->height, G->height);
			C->height = 1 + Max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb.Combine(B->aabb, F->aabb);
			C->aabb.Combine(A->aabb, G->aabb);

			A->height = 1 + Max(B->height, F->height);
			C->height = 1 + Max(A->height, G->height);
		}

		return iC;
	}

	// Rotate B up
	if (balance < -1)
	{
		int iD = B->child1;
		int iE = B->child2;
		b2TreeNode* D = m_nodes + iD;
		b2TreeNode* E = m_nodes + iE;
		assert(0 <= iD && iD < m_nodeCapacity);
		assert(0 <= iE && iE < m_nodeCapacity);

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != b2_nullNode)
		{
			if (m_nodes[B->parent].child1 == iA)
			{
				m_nodes[B->parent].child1 = iB;
			}
			else
			{
				assert(m_nodes[B->parent].child2 == iA);
				m_nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			m_root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb.Combine(C->aabb, E->aabb);
			B->aabb.Combine(A->aabb, D->aabb);

			A->height = 1 + Max(C->height, E->height);
			B->height = 1 + Max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb.Combine(C->aabb, D->aabb);
			B->aabb.Combine(A->aabb, E->aabb);

			A->height = 1 + Max(C->height, D->height);
			B->height = 1 + Max(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

int b2DynamicTree::GetHeight() const
{
	if (m_root == b2_nullNode)
	{
		return 0;
	}

	return m_nodes[m_root].height;
}

//
float b2DynamicTree::GetAreaRatio() const
{
	if (m_root == b2_nullNode)
	{
		return 0.0f;
	}

	const b2TreeNode* root = m_nodes + m_root;
	float rootArea = root->aabb.GetPerimeter();

	float totalArea = 0.0f;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const b2TreeNode* node = m_nodes + i;
		if (node->height < 0)
		{
			// Free node in pool
			continue;
		}

		totalArea += node->aabb.GetPerimeter();
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
int b2DynamicTree::ComputeHeight(int nodeId) const
{
	assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2TreeNode* node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		return 0;
	}

	int height1 = ComputeHeight(node->child1);
	int height2 = ComputeHeight(node->child2);
	return 1 + Max(height1, height2);
}

int b2DynamicTree::ComputeHeight() const
{
	int height = ComputeHeight(m_root);
	return height;
}

void b2DynamicTree::ValidateStructure(int index) const
{
	if (index == b2_nullNode)
	{
		return;
	}

	if (index == m_root)
	{
		assert(m_nodes[index].parent == b2_nullNode);
	}

	const b2TreeNode* node = m_nodes + index;

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		assert(child1 == b2_nullNode);
		assert(child2 == b2_nullNode);
		assert(node->height == 0);
		return;
	}

	assert(0 <= child1 && child1 < m_nodeCapacity);
	assert(0 <= child2 && child2 < m_nodeCapacity);

	assert(m_nodes[child1].parent == index);
	assert(m_nodes[child2].parent == index);

	ValidateStructure(child1);
	ValidateStructure(child2);
}

void b2DynamicTree::ValidateMetrics(int index) const
{
	if (index == b2_nullNode)
	{
		return;
	}

	const b2TreeNode* node = m_nodes + index;

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		assert(child1 == b2_nullNode);
		assert(child2 == b2_nullNode);
		assert(node->height == 0);
		return;
	}

	assert(0 <= child1 && child1 < m_nodeCapacity);
	assert(0 <= child2 && child2 < m_nodeCapacity);

	int height1 = m_nodes[child1].height;
	int height2 = m_nodes[child2].height;
	int height;
	height = 1 + Max(height1, height2);
	assert(node->height == height);

	TAABB2 aabb;
	aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

	assert(aabb.GetLowerBound() == node->aabb.GetLowerBound());
	assert(aabb.GetUpperBound() == node->aabb.GetUpperBound());

	ValidateMetrics(child1);
	ValidateMetrics(child2);
}

void b2DynamicTree::Validate() const
{
#if defined(b2DEBUG)
	ValidateStructure(m_root);
	ValidateMetrics(m_root);

	int freeCount = 0;
	int freeIndex = m_freeList;
	while (freeIndex != b2_nullNode)
	{
		assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	assert(GetHeight() == ComputeHeight());

	assert(m_nodeCount + freeCount == m_nodeCapacity);
#endif
}

int b2DynamicTree::GetMaxBalance() const
{
	int maxBalance = 0;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const b2TreeNode* node = m_nodes + i;
		if (node->height <= 1)
		{
			continue;
		}

		assert(node->IsLeaf() == false);

		int child1 = node->child1;
		int child2 = node->child2;
		int balance = abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = Max(maxBalance, balance);
	}

	return maxBalance;
}

void b2DynamicTree::RebuildBottomUp()
{
	int* nodes = (int*)b2Alloc(m_nodeCount * sizeof(int));
	int count = 0;

	// Build array of leaves. Free the rest.
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		if (m_nodes[i].height < 0)
		{
			// free node in pool
			continue;
		}

		if (m_nodes[i].IsLeaf())
		{
			m_nodes[i].parent = b2_nullNode;
			nodes[count] = i;
			++count;
		}
		else
		{
			FreeNode(i);
		}
	}

	while (count > 1)
	{
		float minCost = b2_maxFloat;
		int iMin = -1, jMin = -1;
		for (int i = 0; i < count; ++i)
		{
			TAABB2 aabbi = m_nodes[nodes[i]].aabb;

			for (int j = i + 1; j < count; ++j)
			{
				TAABB2 aabbj = m_nodes[nodes[j]].aabb;
				TAABB2 b;
				b.Combine(aabbi, aabbj);
				float cost = b.GetPerimeter();
				if (cost < minCost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		int index1 = nodes[iMin];
		int index2 = nodes[jMin];
		b2TreeNode* child1 = m_nodes + index1;
		b2TreeNode* child2 = m_nodes + index2;

		int parentIndex = AllocateNode();
		b2TreeNode* parent = m_nodes + parentIndex;
		parent->child1 = index1;
		parent->child2 = index2;
		parent->height = 1 + Max(child1->height, child2->height);
		parent->aabb.Combine(child1->aabb, child2->aabb);
		parent->parent = b2_nullNode;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count - 1];
		nodes[iMin] = parentIndex;
		--count;
	}

	m_root = nodes[0];
	b2Free(nodes);

	Validate();
}

void b2DynamicTree::ShiftOrigin(const TVec2& newOrigin)
{
	// Build array of leaves. Free the rest.
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		m_nodes[i].aabb.GetLowerBound() -= newOrigin;
		m_nodes[i].aabb.GetUpperBound() -= newOrigin;
	}
}
