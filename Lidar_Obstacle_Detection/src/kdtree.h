//kdtree class from quiz folder
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
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
	: root(nullptr)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node*& node, uint depth, std::vector<float> point, int id)
	{
		if(node == nullptr)
		{
			node = new Node(point, id);
		}
		else
		{
			// check current dimension to determine which axis to compare
			int dim = depth % point.size();

			if(point[dim] > node->point[dim])
			{
				insertHelper(node->right, depth+1, point, id);
			}
			else
			{
				insertHelper(node->left, depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	void searchHelper(Node*& node, uint depth, std::vector<float>& target, float distanceTol, std::vector<int>& ids)
	{
		if(node==nullptr)
			return;
		
		bool insideBox = true;
		// distance box check
		for(int i = 0; i < target.size(); ++i)
		{
			if(std::fabs(node->point[i] - target[i]) > distanceTol)
			{
				insideBox = false;
				break;
			}
		}

		if(insideBox)
		{
			float distanceSq{0.0f};
			for(int i = 0; i < target.size(); ++i)
			{
				float diff = node->point[i] - target[i];
				distanceSq += diff * diff;
			}

			if(std::sqrt(distanceSq) <= distanceTol)
				ids.emplace_back(node->id);
		}

		int dim = depth % target.size();
		if ((target[dim] - distanceTol) < node->point[dim])
			searchHelper(node->left, depth + 1, target, distanceTol, ids);
		if ((target[dim] + distanceTol) > node->point[dim])
			searchHelper(node->right, depth + 1, target, distanceTol, ids);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};
