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
public class Talker extends AbstractNodeMain {

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
    final String SOURCE = "http://www.semanticweb.org/ontologies/2016/1/";
    final String test_file = "ontology.owl";
    final String fileName = "ontology_modified.owl";
    final String OWL = ".owl";
    final String TBOX_FILE = "semantic_mapping_domain_model";
    final String ABOX_FILE = "semantic_map1";
    final String absoluteFileName = JENA + fileName;
    final String NS = SOURCE + TBOX_FILE + "#";
    

    /**
      * Tbox and Abox
      */
    OntModel tbox = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM );
    OntDocumentManager dm_tbox = tbox.getDocumentManager();
    dm_tbox.addAltEntry(SOURCE+ TBOX_FILE,"file:" + JENA + TBOX_FILE + OWL );
    tbox.read(SOURCE+TBOX_FILE,"RDF/XML");

    OntModel abox = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM );
    OntDocumentManager dm_abox = abox.getDocumentManager();
    dm_abox.addAltEntry(SOURCE+ABOX_FILE,"file:" + JENA + ABOX_FILE + OWL );
    //dm_abox.setProcessImports( false );
    abox.read(SOURCE+ABOX_FILE,"RDF/XML");
    
    /** 
     * Instead of creating an inference ontology model
     * based on only the tBox, we will create a specialized 
     * reasoner on tbox and apply inferences on abox.
     * Thus, instead of:
     *  OntModel inf = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM_MICRO_RULE_INF, base);
     */

    Reasoner reasoner = ReasonerRegistry.getOWLReasoner();
    reasoner = reasoner.bindSchema(tbox);
    
    OntModelSpec ontModelSpec = OntModelSpec.OWL_MEM_MICRO_RULE_INF;
    ontModelSpec.setReasoner(reasoner);

    InfModel inf = ModelFactory.createInfModel(reasoner, abox);

    OntClass chairClass = tbox.getOntClass( NS + "Chair" );
    OntClass reskClass = tbox.getOntClass( NS + "Desk" );
    OntClass TableClass = tbox.getOntClass( NS + "Table" );

    System.out.println("\n\n---- Assertions in the data ----\n\n");

    // list the asserted types
    Individual p1 = abox.createIndividual( NS + "chair5", chairClass );
    for (Iterator<Resource> i = p1.listRDFTypes(false); i.hasNext();) {
            System.out.println( p1.getURI()+"\tis asserted in class\t"+ i.next() );
    }

    System.out.println("\n\n---- Inferred Assertions ----\n\n");

    // list the inferred types

    Resource p = inf.getResource( NS + "chair5" );
    System.out.println("chair5 has types:");
    printStatements(inf, p, RDF.type, null);

/*
    for (Iterator<Resource> i=p.listRDFTypes(false); i.hasNext();) {
            System.out.println( p.getURI() + " IS A " + i.next() );
    }

        System.out.println("\n\n---- List all subclasses of Lecturer Class ----\n\n");

        for(Iterator<OntClass> i = lecturerClass.listSubClasses() ; i.hasNext();) {
                OntClass c = i.next();
                System.out.println(c.getURI());
        }

        System.out.println("\n\n---- List all Instances of Lecturer ----\n\n");

        lecturerClass = inf.getOntClass( NS + "Chair" );
        for(ExtendedIterator<? extends OntResource> i = lecturerClass.listInstances() ; i.hasNext();) {
                System.out.println(i.next());
        }

	System.out.println("\n\n---- Consistency Check ----\n\n");
*/

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
