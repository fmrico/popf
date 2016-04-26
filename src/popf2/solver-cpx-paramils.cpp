void setILS(IloCplex * cplex)
{
    if (provided) {
    
        {
            const int pCount = intParams.size();
            for (int p = 0; p < pCount; ++p) {
                cplex->setParam((IloCplex::IntParam) intParams[p].first, intParams[p].second);
            }
        }
        
        {
            const int pCount = boolParams.size();
            for (int p = 0; p < pCount; ++p) {
                cplex->setParam((IloCplex::BoolParam) boolParams[p].first, boolParams[p].second);
            }
        }
        
        {
            const int pCount = numParams.size();
            for (int p = 0; p < pCount; ++p) {
                cplex->setParam((IloCplex::NumParam) numParams[p].first, numParams[p].second);
            }
        }
        
        return;
    }

    
}

