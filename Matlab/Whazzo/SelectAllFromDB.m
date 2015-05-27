function alldata = SelectAllFromDB(conn,tableName)

if isconnection(conn)
qry = sprintf('Select * From %s;',tableName);
display(qry);
rs = fetch(exec(conn, qry));
alldata = get(rs, 'Data');
display(alldata);
else
display('MySql Connection Error');
end