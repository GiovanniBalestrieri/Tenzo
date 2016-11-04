/** 

The Vendor class allows the creation of a shop owner instance.
Each vendor has a predetermined number of son. 
*/

class Vendor{

/*
//will get memory only once and retain its value
static int count = 0; 

// static block:
// Is used to executed code before main method at the time of class loading
static{
System.out.println("Creating new Vendor");
}
*/
private int id;
public int[] sons;

Vendor(int v_id){
	id = v_id;
	sons = new int[5];
	//count++;
}

/*
static public void printTotalNumberOfVendor(){
	System.out.println("There are " + count + " vendors connected to the network");
}
*/

public int getId(){
	return this.id;
}



}
