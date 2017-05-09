#ifndef _OPTIMISER
#define  _OPTIMISER

#include "nvec.h"

#define MAX_ITER 5000 
class Candidate ;

class Model
{
public:
	nvec UBound ;
	nvec LBound ;
	int numFnCalls ;

	Model(){} ;
	virtual void  generateCandidate( Candidate &C ){} ;
	virtual void  evaluate( Candidate &C ) {} ;
	virtual void  keepWithinBounds( Candidate &C  );
};

class Candidate
{
public:
	float score , actual_score ;
	float pb_Score ;
	nvec P ;
	nvec V ;
	nvec PB ;

	Candidate()
	{
		score = actual_score = pb_Score = 0.0 ;
	}


	void updatePB( bool maximise = true )
	{
		//cout << maximise << endl ;
		if( maximise == true )
		{
			if( score > pb_Score)
			{
				pb_Score  = score ;
				PB = P ;
			}
		}
		else
		{
			if( score < pb_Score)
			{
				pb_Score  = score ;
				PB = P ;
			}
			//cout << " minimise" << endl ;
		}
	}
	void update( Candidate &GB , float &w , float &wp , float & wg)
	{
		// update Vel
		float rp = ofRandom(0,1);
		float rg = ofRandom(0,1);
		V = V * w + (PB - P) * wp * rp + ( GB.P - P ) * wg * rg ;		
		//update Pos
		P += V ;
	}



};


void  Model::keepWithinBounds(Candidate &C )
{
	// keep current position within search domain ;
	for( int i = 0  ; i < C.P.size() ; i++ )  C.P.x[i] = ofClamp(  C.P.x[i]  , LBound.x[i] , UBound.x[i] ) ;


}


class Optimiser 
{
public:

	Candidate *pop ;	
	int numC ;

	Candidate GB ;
	float bestScore ;

	bool maximise ;
	float min , max ;
	int iteration ;

	// pso
	float w,wp,wg;
	#define updateGB {bestScore = pop[i].score ;GB = pop[i];} 

	// ga
	Candidate *mating ;

	//gsa
	float G ;
	nvec F;
	nvec f;
	nvec A;

	/// history & perf. stats
	Candidate *searchSpace ;
	int maxIterations ;
	int numSearches ;

	////////////////////////////////////////////////////////////////////////// CONSTRUCTORS 
	
	Optimiser(){};
	Optimiser( int _numC , Model &M , float _w = 1, float _wp = 3.0 , float _wg = 1.0 , bool _maximise = true )
	{
		// --------------- attribs

		iteration = 0 ;
		numSearches = 0 ;
		maxIterations = MAX_ITER ;
		numC = _numC ;
		w = _w ;wp = _wp ; wg = _wg ;
		maximise = _maximise ;

		// ---------------  mem alloc
		// history  
		searchSpace = new Candidate[ numC * maxIterations ];
		for( int i =0 ; i < numC * maxIterations ; i++ )
		{
			searchSpace[i] = *new Candidate() ;
			M.generateCandidate( searchSpace[i] ) ;
		}
		
		// iteration 
		pop = new Candidate[numC] ;
		mating = new Candidate[numC] ;
		for( int i =0 ; i < numC ; i++ )
		{
			pop[i] = *new Candidate() ;
			M.generateCandidate( pop[i] ) ;

			mating[i] = *new Candidate() ;
			M.generateCandidate( mating[i] ) ;
		}

		// --------------- initialise
		bestScore = pow(20.0,20.0);		
		if( maximise ) bestScore *= -1 ;
		
		GB = * new Candidate() ;

		GB.P  = nvec(pop[0].P.size());
		GB.PB = nvec(pop[0].P.size());
		GB.V  = nvec(pop[0].P.size());
		GB.score = bestScore ;

		//  GSA
		G = 200.000 ;
		F = nvec(pop[0].P.size());
		A = nvec(pop[0].P.size());
		f = nvec(pop[0].P.size());

	}

	
	////////////////////////////////////////////////////////////////////////// UTILITIES  
	
	void reset( Model &M )
	{
		for( int i =0 ; i < numC ; i++ )M.generateCandidate( pop[i] ) ;

		bestScore = pow(20.0,20.0);		
		if(maximise) bestScore *= -1 ;
		
		GB.P = 0.0 ; 
		GB.score = bestScore ;

		iteration = 0 ;
		M.numFnCalls = 0 ;

		numSearches = 0 ;

	}
	void updateStats()
	{
		min = pow(10.0,20.0);max = -min ;
		float sum = 0.0 ;
		for( int i =0 ; i < numC ; i++ )
		{
			if( pop[i].score < min )min = pop[i].score ;
			if( pop[i].score > max )max = pop[i].score ;
		}

		// update GB
		for( int i =0 ; i < numC ; i++ )
		{
			if( maximise && pop[i].score  > bestScore )updateGB
			if( !maximise && pop[i].score < bestScore )updateGB
		}
	}
	void appendToSearchSpace()
	{
		for( int i =0 ; i < numC ; i++ )
			if( numSearches < numC * maxIterations )
			{
				searchSpace[numSearches] = pop[i] ;
				numSearches ++ ;
			}
	}
	
	////////////////////////////////////////////////////////////////////////// COMPUTE 

	void update_pso( Model &M , bool _maximise = true  )
	{
		iteration ++ ;
		
		// keep candidates within search space ;
		for( int i =0 ; i < numC ; i++ )M.keepWithinBounds(pop[i]);
		
		// score candidates & update PB if needed ;
		for( int i =0 ; i < numC ; i++ )
		{
			M.evaluate( pop[i] ) ;
			pop[i].updatePB( _maximise ) ;
		}
		
		// update GB ;		
		for( int i =0 ; i < numC ; i++ )
		{
			if( _maximise && pop[i].score  > bestScore )updateGB
			if( !_maximise && pop[i].score < bestScore )updateGB
		}
		
		// for each particle , update V & P ;
		for( int i =0 ; i < numC ; i++ )pop[i].update(GB,w,wp,wg) ;

		appendToSearchSpace();

	}
	void update_grad( Model &M , Candidate &C , bool analytical )
	{
		iteration++ ;

		nvec grad = nvec(C.P.size());
		float curScore = C.score ;

		M.keepWithinBounds(C);

		for( int i =0 ; i < C.P.size() ; i++ )
		{
			C.P.x[i] -= 0.1 ;
			M.evaluate(C);
			float val_AtMinus = C.score ;
			C.P.x[i] += 0.2 ;
			M.evaluate(C);
			float val_AtPlus = C.score ;
			C.P.x[i] -= 0.1;

			float deltaP = 0.2 ;
			grad.x[i] = (val_AtPlus - val_AtMinus) / deltaP ;
			
		}

		float d = grad.mag() ;
		grad.normalise();
		grad *= /*maximise ? 0.1*d :*/ -0.1*d ;

		C.P += grad ;

		if( numSearches < numC * maxIterations )
		{
			searchSpace[numSearches] = GB ;
			numSearches++;
		}
		/*M.evaluate(C) ;
		if( curScore < C.score )C.P-=grad ;

		C.P -= grad ;
		M.evaluate(C) ;
		if( curScore < C.score )C.P+=grad ;*/

		//for( int i =0 ; i < grad.X.size() ; i++ )C.P.X[i] +=  grad.X[i] ;
	}

	void offSpring( Candidate &A , Candidate &B , Candidate &Child )
	{
		for( int i =0 ; i < Child.P.size() ; i++ )
			//Child.P.x[i] = ( i > float(Child.P.size()) * 0.5 ) ? A.P.x[i] : B.P.x[i]  ;
		Child.P.x[i] = ( i % 2 == 0 /*> float(Child.P.size()) * 0.5*/ ) ? A.P.x[i] : B.P.x[i]  ;
		
	}

	void update_ga( Model &M , bool maximise = true )
	{
		iteration ++ ;
		float mutateRate = 0.1 * (1.0 - float(iteration)/float(MAX_ITER)) ; 
		
		// keep candidates within search space ;
		for( int i =0 ; i < numC ; i++ )M.keepWithinBounds(pop[i]);

		//evaluate candidate pop.ln
		for( int i =0 ; i < numC ; i++ )M.evaluate(pop[i]);
		
		// update min max & GB ;
		updateStats() ;

		// ----------------------- next gen 
		if(!maximise)for( int i =0 ; i < numC ; i++ )pop[i].score = max / pop[i].score ;

		// sum  scores
		float total = 0.0 ;
		for( int i =0 ; i < numC ; i++ ) total += pop[i].score ;
		
		// percentile scores ;
		for( int i =0 ; i < numC ; i++ ) pop[i].score /= total ; 
		
		// --- probabilistic mating pool 
		int cnt = 0 ;		
		for( int i = 0 ; i < numC ; i++ )
			for( int j =0 ; j < int( pop[i].score * float(numC)+ 0.5 ) ; j++ , cnt++ ) // + 0.5 -> round to nearest int ;
						if( cnt < numC ) mating[cnt] = pop[i] ;

		//fill remainder with GB ;
		for( int i = cnt ; i < numC ; i++ )mating[i] = GB ;

		// --- create new generation ;
		#define rn int(ofRandom(0,numC) )
		for( int i =0 ; i < numC ; i++ )
		{
			offSpring(mating[rn],mating[rn],pop[i]) ;
			//mutate
			for( int i =0 ; i < pop[i].P.size() ; i++ )
				if(ofRandom(0,1) < mutateRate ) pop[i].P.x[i] = ofRandom( M.LBound.x[i] , M.UBound.x[i] ) ;
		}

		appendToSearchSpace();

	}
	void update_gsa( Model &M , bool maximise = true )
	{
		
		iteration ++ ;
		float Gcur =  G * pow( ( MAX_ITER / iteration ) , 0.75 ) ;// G *( 1 - (iteration / 1200));
		float multiplier = 1; //10 * ( 1 - (iteration / 1200)) + 1 ;
		
		// keep candidates within search space ;
		for( int i =0 ; i < numC ; i++ )M.keepWithinBounds(pop[i]);

		//------------------ evaluate candidate pop.ln	; score = F(mass) in this alogrithm ;	
		for( int i =0 ; i < numC ; i++ )M.evaluate(pop[i]);

		//------------------ calculate mass via fitness 

		// update min max & GB ;
		updateStats() ;

		if( fabs(max - min )< 1e-8 )max+=1e-8 ;
		if( !maximise )swap(min,max);

		// calc total of scores 
		float sum = 0 ;
		for( int i =0 ; i < numC ; i++ )sum += pop[i].score ;
		
		// (normalized)inertial masses
		for( int i =0 ; i < numC ; i++ )pop[i].score = ( (pop[i].score - min )/(max-min)) / sum * multiplier;

		//------------------ calc gravity force, acceleration and update position :
		
		float m1,m2,r;

		// create dup. array of pop		
		for( int i =0 ; i < numC ; i++ )mating[i] = pop[i] ;

		// calc forces
		nvec dir( pop[0].P.size());
		for( int i =0 ; i < numC ; i++ )
		{
			m1 = pop[i].score + 1e-16 ;
			F = 0 ;

			for( int j =0 ; j < numC ; j++ )
			{
					if( i == j )continue ;
				
				m2 = pop[j].score ;
				dir = pop[j].P ;
				dir -= pop[i].P ;
				f  = dir*(Gcur*m1*m2);	
				f /= dir.mag() + 0.1;
				f *= ofRandom(0,1) ;
				F += f ;
			}
			
			A = F / m1  ; 
			mating[i].V  = (mating[i].V * ofRandom(0,0.1)) ; //+ A ;
			mating[i].V += A ;
			mating[i].P += mating[i].V ;
		}

		// swap pop arrays
		for( int i =0 ; i < numC ; i++ )pop[i] = mating[i] ;

		appendToSearchSpace();
	}
	

};




#endif
