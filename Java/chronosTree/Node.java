/** 
The Node class allows the creation of a user instance with maximum 3 sons 

node_id : identification number for the node
userfather_id : id of the father, father can be AlphaNode or Node3
soni_id : id of the invited sons
generation level of depth of the generated tree.

*/

class Node {

static int node_count = 0;
private int node_id;
public int father_id;
public int son1_id;
public int son2_id;
public int son3_id;

public String generation;
static{
System.out.println("Creating new Node");
}


Node(int id){
	this.node_id = id;
	node_count++;
	System.out.println("Tot nodes: "+ node_count);
}

Node(int father,String generation){
	father_id = father;
	this.generation = generation;
}

Node(int id, int f_id, String gen){
	this(f_id,gen);
	this.node_id = id;
}

public void setChildren(int s1, int s2, int s3){
	son1_id = s1;
	son2_id = s2;
	son3_id = s3;
}

public int getFatherId(){
	return father_id;
}

public int getId(){
	return node_id;
}

public int getFirstSonId(){
	return son1_id;
}


public int getSecondSonId(){
	return son2_id;
}


public int getThirdSonId(){
	return son3_id;
}

}
