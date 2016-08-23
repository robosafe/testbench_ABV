import re
import sys

def legible_traces(traces_file,model_file):
# -------------- PARSING OF THE MODEL INTO USEFUL STRUCTURES
	variables_i=[]
	variables_e=[]
	automata_i=[]
	automata_e=[]
	transition_i=[]
	transition_e=[]
	for i, line in enumerate(open(model_file+'.xml', 'r')):
		for match in re.finditer("<declaration>", line): # Global variables
			variables_i.append(i+1)
    		for match in re.finditer("</declaration>", line):
			variables_e.append(i+1)
		for match in re.finditer("<template>", line): # All automata
			automata_i.append(i+1)
		for match in re.finditer("</template>", line):
			automata_e.append(i+1)
		for match in re.finditer("<transition>", line):
			transition_i.append(i+1)
		for match in re.finditer("</transition>", line):
			transition_e.append(i+1)
#	print variables_i
#	print variables_e
#	print automata_i
#	print automata_e
#	print transition_i
#	print transition_e
	global_variables = []
	for i, line in enumerate(open(model_file+'.xml', 'r')): # All global variables
		if i>variables_i[0] and i<variables_e[0]:
			 if re.match("int", line, flags=0) != None:
			 	variable_name = re.split("int ",line)
			 	variable_name = re.split("[//]",variable_name[1])
			 	variable_name = re.split(";",variable_name[0])
			 	global_variables.append(variable_name[0])
			 if re.match("bool", line, flags=0) != None:
			 	variable_name = re.split("bool ",line)
			 	variable_name = re.split("[//]",variable_name[1])
			 	variable_name = re.split(";",variable_name[0])
			 	global_variables.append(variable_name[0])
#	print global_variables
			 
	names_automata=[]
	locname_automata = []
	locid_automata=[]
	for j, element in enumerate(automata_i):
		locid_automaton=[]
		locname_automaton=[]
		for i, line in enumerate(open(model_file+'.xml', 'r')): # Parse each automata
			if i==automata_i[j]:
				for match in re.finditer("\t\t<name>", line): #Find start of names of automata
					name_automata = re.split("\t\t<name>",line)
					name_automata = re.split("</name>",name_automata[1])
					names_automata.append(name_automata[0])
			if i>automata_i[j] and i<automata_e[j]:
				for match in re.finditer("<location id=\"",line): #Find start of all locations of all automata
					location_id = re.split("<location id=\"",line)
					location_id = re.split("\"",location_id[1])
					locid_automaton.append(location_id[0])
				for match in re.finditer("<name x=\"",line):
					location_name = re.split("<name x=\"",line)
					location_name = re.split("\">",location_name[1])
					location_name = re.split("</name>",location_name[1])
					locname_automaton.append(location_name[0])
		locname_automata.append(locname_automaton)
		locid_automata.append(locid_automaton)
#	print names_automata
# 	print locname_automata
#	print locid_automata
	tran_automata=[]
	count_automata=0
	tran_automaton=[]
	for k, trans in enumerate(transition_i): # Parse all transitions per automaton
		transitions=[]
		transition=['0','0','0']
		for i, line in enumerate(open(model_file+'.xml', 'r')): 
			if i>=transition_i[k] and i<=transition_e[k]:
				for match in re.finditer("source ref=\"",line): 
					source_ref = re.split("source ref=\"",line) 
					source_ref = re.split("\"",source_ref[1])
			        	transitions.append(source_ref[0])
				for match in re.finditer("target ref=\"",line):
					target_ref = re.split("target ref=\"",line)
					target_ref = re.split("\"",target_ref[1])
					transitions.append(target_ref[0])
				for match in re.finditer("label kind=\"guard\"",line):
					guards = re.split("label kind=\"guard\"",line)
					guards = re.split("\">",guards[1])
					guards = re.split("</label>",guards[1])
					transition[0]=guards[0]
				for match in re.finditer("label kind=\"synchronisation\"",line):
					synchs = re.split("label kind=\"synchronisation\"",line)
					synchs = re.split("\">",synchs[1])
					synchs = re.split("</label>",synchs[1])
					transition[1]=synchs[0]
				for match in re.finditer("label kind=\"assignment\"",line):
					assigns = re.split("label kind=\"assignment\"",line)
					assigns = re.split("\">",assigns[1])
					assigns = re.split("</label>",assigns[1])
					transition[2]=assigns[0]
		transitions.append(transition)
		if trans>=automata_i[count_automata] and transition_e[k]<=automata_e[count_automata]:			
			tran_automaton.append(transitions)
		elif transition_i[k]>=automata_i[count_automata+1] and transition_e[k]<=automata_e[count_automata+1]:
			tran_automata.append(tran_automaton)
			tran_automaton=[]
			tran_automaton.append(transitions)
			count_automata+=1
		if k==(len(transition_i)-1) and count_automata==(len(names_automata)-1):
			tran_automata.append(tran_automaton)
			
#	print tran_automata


#--------------------- TRANSLATING THE TRACES FILE
# in the file: no_automata states . x . x . x . x . x . x. x .. no_variables . transitions . [repeat]
	f = open('legible'+ traces_file+'.tr', 'w')
	no_automata = len(names_automata)
	no_variables = len(global_variables)
	all_lines=[]
	for i, line in enumerate(open(traces_file+'.xtr', 'r')): 
		#if re.match("\n", line, flags=0) != None:
		c_line = re.split("\n",line)
		all_lines.append(c_line[0])
#	print all_lines
	state_0 = []
	
	for i in range(0,no_automata):
		state_0.append(all_lines.pop(0))
	for i in range(0,30): #Modify according to model
		all_lines.pop(0)
	for i in range(0,len(global_variables)):
		state_0.append(all_lines.pop(0))
#	print state_0
	f.write("Initial state:\n")
	variables_count=0
	for i in range(0,no_automata):
			automaton_names = locname_automata[i]
			f.write(names_automata[i] + '.' + automaton_names[int(state_0[i])]+'  ')
	for i in range(0,len(global_variables)):
			variable_name = re.split("=",global_variables[variables_count])
			f.write(variable_name[0] + '=' + state_0[i+no_automata]+'  ')
			variables_count+=1
	# print automaton_names
	transitions_1=[]
	state_1=[]
	while len(all_lines)>2:
		all_lines.pop(0) #pop first . 
		for i in range(0,no_automata):
			state_1.append(all_lines.pop(0))
		
		for i in range(0,30): #Modify according to model
			all_lines.pop(0)
		for i in range(0,len(global_variables)):
			state_1.append(all_lines.pop(0))
		#print state_1
		all_lines.pop(0) #extra . between variables and transitions
		for i in range(0,no_automata):
			if re.search(" ", all_lines[0],flags=0) != None:
				transitions_1.append(all_lines.pop(0))
		#print transitions_1
		f.write("\n\nTransitions:\n")
		for i in range(0,len(transitions_1)):
			automaton_t = re.split(" ",transitions_1[i])# Transition in the trace
			all_tran_aut=tran_automata[int(automaton_t[0])] #All transitions of the automaton
			particular_tran = all_tran_aut[int(automaton_t[1])-1] #Finding particular transition
			origin_state = particular_tran[0]
			destin_state = particular_tran[1]
			process_strings = particular_tran[2] #Translate to legible characters
			all_locid_aut = locid_automata[int(automaton_t[0])] #All the locid of the automaton
			all_locname_aut = locname_automata[int(automaton_t[0])] #All the locname of the automaton
			for k,locid in enumerate(all_locid_aut):
				if locid == origin_state:
					origin_name = all_locname_aut[k]
				if locid == destin_state:
					destin_name = all_locname_aut[k]
			f.write(names_automata[int(automaton_t[0])]+'.'+origin_name+ ' -> '+names_automata[int(automaton_t[0])]+'.'+destin_name+' {'+corrected(process_strings[0])+'; '+corrected(process_strings[1])+'; '+corrected(process_strings[2])+'}\n')
		variables_count=0
		f.write("\nState:\n")
		for i in range(0,no_automata):
			automaton_names = locname_automata[i]
			f.write(names_automata[i] + '.' + automaton_names[int(state_1[i])]+'  ')
		for i in range(0,len(global_variables)): 
			variable_name = re.split("=",global_variables[variables_count])
			f.write(variable_name[0] + '=' + state_1[i+no_automata] + '  ')
			variables_count+=1
		state_1 = []
		transitions_1 = []
		
	

#token_pat = re.compile("(?:(\w+)|(\+)|(\-)|(\=*)|(.amp;.amp)|([|][|])|([:]=)|(.gt;=*)|(.lt;=*)|([!])|([?])|([,])|([(])|([)]))")

def corrected(expr):
#	print expr
	expr_new=''
	modif1 = re.split("&amp;&amp;",expr)
	expr_new=expr_new+modif1[0]
    	for l in range(1,len(modif1)):
    		expr_new=expr_new+' && '+ modif1[l]
    	modif2 = re.split("&gt;",expr_new)
    	expr_new=''
	expr_new=expr_new+modif2[0]
    	for l in range(1,len(modif2)):
    		expr_new=expr_new+' > '+ modif2[l]
    	modif3 = re.split("&gt;=",expr_new)
    	expr_new=''
	expr_new=expr_new+modif3[0]
    	for l in range(1,len(modif3)):
    		expr_new=expr_new+' >= '+ modif3[l]
    	modif4 = re.split("&lt;",expr_new)
    	expr_new=''
	expr_new=expr_new+modif4[0]
    	for l in range(1,len(modif4)):
    		expr_new=expr_new+' < '+ modif4[l]
    	modif5 = re.split("&lt;=",expr_new)
    	expr_new=''
	expr_new=expr_new+modif5[0]
    	for l in range(1,len(modif5)):
    		expr_new=expr_new+' <= '+ modif5[l]
 #   	print expr_new
	return expr_new


if __name__ == "__main__":
	if len(sys.argv) == 3: #arguments passed by command line: program, trace file, xml file
		legible_traces(sys.argv[1],sys.argv[2])
	else:
		print 'legible_traces [trace file or .xtr] [model or .xml]'
		sys.exit(1)

