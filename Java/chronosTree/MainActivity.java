
class MainActivity{


public static void main(String args[]){

	Vendor first = new Vendor(0);
	int fatherId = first.getId();
	Node3 test = new Node3(first.getId(),"alpha");
	System.out.println("Node  father id: "+ test.getFatherId());
}	

}
