 conn = database('whazzo','userk','whazzart','com.mysql.jdbc.Driver','jdbc:mysql://localhost:3306/whazzo')
 
   sqlquery = ['SELECT name FROM generi;'];
   curs = exec(conn, sqlquery);
   curs = fetch(curs);
   generi = curs.data;
   generi(4,1)
   
%%
conn = database('matlab','root','',['/Library/Java/Extensions/'...
    'mysql-connector-java-5.1.22-bin.jar'],'jdbc:mysql://localhost/');
qtimeout = 3;
 
% What you want to import (by default the whole column is imported)
securities = {'TSLA','AAPL','SPY'};
timeformat = 'd';
fieldname = 'adj close';
 
% Do for each security individually
[~,n] = size(securities);
for i = 1:n
   symbol = securities{1,i};
 
   % First go to _Symbols to lookup the table for the security
   setdbprefs('DataReturnFormat','cellarray');
   sqlquery = ['SELECT nvTableName FROM _Symbols WHERE nvSymbol=''',symbol,''''];
   curs = exec(conn, sqlquery, qtimeout);
   curs = fetch(curs);
   nvTableName = curs.data{1,1};
 
   % Second, go to _NamesColumn to lookup the column name
   setdbprefs('DataReturnFormat','cellarray');
   sqlquery = ['SELECT ',timeformat,' FROM _NamesColumn WHERE PK_nvColumnID=''',fieldname,''''];
   curs = exec(conn, sqlquery, qtimeout);
   curs = fetch(curs);
   nvColumnName = curs.data{1,1};
 
   % Third, go to the nvTableName and retrieve ALL the data for the time
   % and fieldname
   setdbprefs('DataReturnFormat','numeric');
   sqlquery = ['SELECT PK_inDate, ',nvColumnName,' FROM ',nvTableName,' ORDER BY PK_inDate ASC'];
   curs = exec(conn, sqlquery, qtimeout);
   curs = fetch(curs);
 
   if i==1
       [rows,~] = size(curs.data); % get the number of rows from the data retrieved
       import = zeros(rows,n+1); % create empty vector for the data
       symbols = cell(1,n+1); % save the names of the securities, first column is date
       symbols{1,1} = 'Date';
       symbols{1,i+1} = symbol;
       import(:,1) = curs.data(:,1); % we need ( ) because we retrieved it numerically, write timedata
       import(:,2) = curs.data(:,2); % write the imported data for first symbol
   else
       % more than one symbol is retrieved. add the columns after the ones
       % already retrieved. Check for missing data!
       [rows,~] = size(curs.data);
       symbols{1,i+1} = symbol; %create the symbols cell which contains the names of the columns
       for k = 1:rows
           if curs.data(k,1) == import(k,1) % only works if retrieved timeframe is the same, default case
               import(k,i+1) = curs.data(k,2);
           else
               % data is missing or something is shifted. This loop can
               % slow down things and should not happen if the data is
               % synchronus
               print = ['Data for time ',num2str(import(k,1)),' is missing for symbol ',symbol,'\n'];
               fprintf(1,print);
               [rows2,~] = size(import);
               for l =  1:rows2 
                   % search for the right time enty, taking the data that already exists as the base case
                   % the base case is always the very first symbol that was
                   % retrieved from the database. Go through all the rows
                   % and see whether one date entry matches to the current
                   % symbol that is being imported
                   if import(l,1) == curs.data(k,1)
                       import(l,i+1) = curs.data(k,2);                       
                   end
               end
           end
       end
   end
 
end
 
clearvars -except import symbols

