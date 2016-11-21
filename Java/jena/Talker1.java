/*
 * Copyright (C) 2014 userk.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.huric.my_pub_sub_tutorial;

import java.util.Iterator;
import org.apache.jena.util.FileManager;
import org.apache.jena.util.PrintUtil;
import org.apache.jena.util.iterator.ExtendedIterator;
import org.apache.jena.ontology.Individual;
import org.apache.jena.rdf.model.*;
import org.apache.jena.vocabulary.*;
import org.apache.jena.ontology.Ontology;
import org.apache.jena.ontology.OntResource;
import org.apache.jena.ontology.OntModel;
import org.apache.jena.ontology.OntClass;
import org.apache.jena.ontology.OntModelSpec;

import java.io.FileWriter;

import org.apache.jena.reasoner.Reasoner;
import org.apache.jena.reasoner.ReasonerRegistry;
import org.apache.jena.reasoner.ValidityReport;
import org.apache.jena.reasoner.ValidityReport.Report;
import org.apache.jena.ontology.OntDocumentManager;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;


import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class Talker1 extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/talker");
  }
  
  public void printStatements(Model m, Resource s, Property p, Resource o) {
    for (StmtIterator i = m.listStatements(s,p,o); i.hasNext(); ) {
        Statement stmt = i.nextStatement();
        System.out.println(" - " + PrintUtil.print(stmt));
    }
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
 
    final String JENA = "/home/userk/catkin_ws/rosjava/src/huric/my_pub_sub_tutorial/src/main/java/com/github/huric/my_pub_sub_tutorial/";
    final String SOURCE = "urn:x-hp:eg";
    final String NS = SOURCE + "#";
    
    /**
      * Tbox and Abox
      */
    
	OntModel schema = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM );
	OntDocumentManager dm_tbox = schema.getDocumentManager();
	dm_tbox.addAltEntry(SOURCE,"file:" + JENA + "owlDemoSchema.owl");
	schema.read(SOURCE,"RDF/XML");
	

	OntModel data = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM);
	OntDocumentManager dma = data.getDocumentManager();
	dma.addAltEntry(SOURCE,"file:"+JENA+"owlDemoData.rdf");
	data.read(SOURCE,"RDF/XML");

	Reasoner reasoner = ReasonerRegistry.getOWLReasoner();
	reasoner = reasoner.bindSchema(schema);
	OntModelSpec ontModelSpec=OntModelSpec.OWL_MEM_MICRO_RULE_INF;
	ontModelSpec.setReasoner(reasoner);
	InfModel infmodel = ModelFactory.createInfModel(reasoner,data);
    /** 
     * Instead of creating an inference ontology model
     * based on only the tBox, we will create a specialized 
     * reasoner on tbox and apply inferences on abox.
     * Thus, instead of:
     *  OntModel inf = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM_MICRO_RULE_INF, base);
     */


    System.out.println("\n\n-- All you know about an instance --\n\n");

	/* Find out all we know about an instance */
	
	Resource nForce = infmodel.getResource("urn:x-hp:eg/nForce");
	System.out.println("nForce *:");
	printStatements(infmodel, nForce, null, null);


    System.out.println("\n\n---- Check if is an instance of a class ----\n\n");
	/*Test if an individual is an instance of a class expression*/
	
	Resource gamingComputer = infmodel.getResource("urn:x-hp:eg/GamingComputer");
	Resource whiteBox = infmodel.getResource("urn:x-hp:eg/whiteBoxZX");
	if (infmodel.contains(whiteBox, RDF.type, gamingComputer)) {
	    System.out.println("White box recognized as gaming computer");
	} else {
	    System.out.println("Failed to recognize white box correctly");
	}

    System.out.println("\n\n---- Consistency Check ----\n\n");
	
	/* Consistency Check */
	ValidityReport validity = infmodel.validate();
	if (validity.isValid()) {
	    System.out.println("OK");
	} else {
	    System.out.println("Conflicts");
	    for (Iterator i = validity.getReports(); i.hasNext(); ) {
	        ValidityReport.Report report = (ValidityReport.Report)i.next();
	        System.out.println(" - " + report);
	    }
	}

    System.out.println("\n\n---- END ----\n\n");

    final Publisher<std_msgs.String> publisher =
        connectedNode.newPublisher("chatter", std_msgs.String._TYPE);
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        std_msgs.String str = publisher.newMessage();
        str.setData("Hello world! " + sequenceNumber);
        publisher.publish(str);
        sequenceNumber++;
        Thread.sleep(1000);
      }
    });
  }
}
