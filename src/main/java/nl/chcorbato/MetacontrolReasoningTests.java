package nl.chcorbato;

import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.dlsyntax.renderer.DLSyntaxObjectRenderer;
import org.semanticweb.owlapi.formats.PrefixDocumentFormat;
import org.semanticweb.owlapi.io.OWLObjectRenderer;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLAxiom;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLLiteral;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;

import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;
import com.clarkparsia.owlapi.explanation.util.SilentExplanationProgressMonitor;
import com.clarkparsia.pellet.owlapiv3.PelletReasonerFactory;

import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationOrderer;
import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationOrdererImpl;
import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationTree;
import uk.ac.manchester.cs.owl.explanation.ordering.Tree;

/**
 * @author chcorbato
 *
 */
public class MetacontrolReasoningTests {
	private static final String ONTOLOGY_FILE = "file:/home/chcorbato/mros_ws/tomasys/metacontrol_unexmin.owl";
	private static final String ONTOLOGY_IRI = "http://www.semanticweb.org/chcorbato/ontologies/2018/metacontrol_unexmin";
	private static OWLObjectRenderer renderer = new DLSyntaxObjectRenderer();
	
	public static void main(String[] args) throws OWLOntologyCreationException {
		
		//prepare ontology and reasoner
        OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
        OWLOntology ontology = manager.loadOntologyFromOntologyDocument(IRI.create(ONTOLOGY_FILE));
        OWLReasonerFactory reasonerFactory = PelletReasonerFactory.getInstance();
        OWLReasoner reasoner = reasonerFactory.createReasoner(ontology, new SimpleConfiguration());
        OWLDataFactory factory = manager.getOWLDataFactory();
        PrefixDocumentFormat pm = manager.getOntologyFormat(ontology).asPrefixOWLOntologyFormat();
        pm.setDefaultPrefix(ONTOLOGY_IRI + "#");
		
        //get tomasys classes
        OWLClass componentState = factory.getOWLClass(":ComponentState", pm);
        OWLClass componentClass = factory.getOWLClass(":ComponentClass", pm);
        OWLClass function = factory.getOWLClass(":Function", pm);
        OWLClass objective = factory.getOWLClass(":Objective", pm);
        OWLClass functionDesign = factory.getOWLClass(":FunctionDesign", pm);
        OWLClass functionGrounding = factory.getOWLClass(":FunctionGrounding", pm);
                
        ontology = updateComponentStates(manager, ontology, reasoner, factory, pm); // TODO replace with real update using ROS introspection
		
        //get values of selected properties on the individual
        OWLDataProperty o_status = factory.getOWLDataProperty(":o_status", pm);
        OWLNamedIndividual o_move_fwd = factory.getOWLNamedIndividual(":o_move_fwd", pm);

        
        // get value for objective states
        for (OWLLiteral ind : reasoner.getDataPropertyValues(o_move_fwd, o_status)) {
            System.out.println("objective " + renderer.render(o_move_fwd) + " status= " + ind.getLiteral());
        }
        
        //update 
        OWLDataPropertyAssertionAxiom axiomToExplain = factory.getOWLDataPropertyAssertionAxiom(o_status, o_move_fwd, false);
        
        //get explanation
//        DefaultExplanationGenerator explanationGenerator =
//                new DefaultExplanationGenerator(
//                        manager, reasonerFactory, ontology, reasoner, new SilentExplanationProgressMonitor());
//        Set<OWLAxiom> explanation = explanationGenerator.getExplanation(axiomToExplain);
//        ExplanationOrderer deo = new ExplanationOrdererImpl(manager);
//        ExplanationTree explanationTree = deo.getOrderedExplanation(axiomToExplain, explanation);
//        System.out.println();
//        System.out.println("-- explanation why Ivan is in class ChildOfMarriedParents --");
//        printIndented(explanationTree, "");
        
	}
	
    private static OWLOntology updateComponentStates(OWLOntologyManager manager, OWLOntology ontology, OWLReasoner reasoner, OWLDataFactory factory, PrefixDocumentFormat pm) {
    	//get tomasys classes and properties
        OWLClass componentState = factory.getOWLClass(":ComponentState", pm);
        
        //get values of selected properties on the individual
        OWLDataProperty c_status = factory.getOWLDataProperty(":c_status", pm);

        OWLDataPropertyAssertionAxiom axiom;
        for (OWLNamedIndividual ind : reasoner.getInstances(componentState, false).getFlattened()) {
        	// componentState update, motor4 in error
			if (ind == factory.getOWLNamedIndividual(":motor4", pm)) 
				axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, false);
				
		    else
		    	axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, true);	

			manager.applyChange(new AddAxiom(ontology, axiom));
			//Ontology is updated
		    reasoner.flush();    		
            // print values given to component statuses
            for (OWLLiteral value : reasoner.getDataPropertyValues(ind, c_status)) {
                System.out.println("Component " + renderer.render(ind) + " status= " + value.getLiteral());
            }
    		
	    }//Close for
            	
    	return ontology; 	
    
    }
    
    /**
     * @author Martin Kuba makub@ics.muni.cz
     */
    private static void printIndented(Tree<OWLAxiom> node, String indent) {
        OWLAxiom axiom = node.getUserObject();
        System.out.println(indent + renderer.render(axiom));
        if (!node.isLeaf()) {
            for (Tree<OWLAxiom> child : node.getChildren()) {
                printIndented(child, indent + "    ");
            }
        }
    }

}
