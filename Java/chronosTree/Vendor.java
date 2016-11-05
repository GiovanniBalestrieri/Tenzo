/** 

The Vendor class allows the creation of a shop owner instance.
Each vendor has a predetermined number of son. 
*/

class Vendor {


//will get memory only once and retain its value
static int count_vendor = 0; 

// static block:
// Is used to executed code before main method at the time of class loading
static{
System.out.println("Creating new Vendor");
}

private int id;
public int[] children;


Vendor(int v_id){
	

	id = v_id;
	children  = new int[5];
	count_vendor++;
	System.out.println("Tot vendors: "+count_vendor); 
}


static public void printTotalNumberOfVendor(){
	System.out.println("There are " + count_vendor + " vendors connected to the network");
}


public int getId(){
	return this.id;
}



}
