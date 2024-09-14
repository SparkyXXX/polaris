/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_cmd.c
 *  Description  : Command Line Control Manager
 *  LastEditors  : Polaris
 *  Date         : 2023-02-07 15:31:59
 *  LastEditTime : 2023-08-15 01:31:24
 */


#include "sys_cmd.h"
#include "lib_str.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

Tree_NodeTypeDef* CurTreeNode;
static char TREE_RET_PATH[NOW_PATH_LENGTH];

static BaseType_t prvTaskStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvPolarCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvParamCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvsumCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvListCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvCdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvGetVariableCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvSetVariableCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t xTaskStats[CMD_COMMAND_NUM] = {
	{"task-stats",
     "\r\ntask-stats:\r\n Displays a table showing the state of each FreeRTOS task\r\n",
     prvTaskStatsCommand,   0},
    {"polar",
     "\r\npolar:\r\n This is a comprehensive mobile interactive robot designed by Syj\r\n",
     prvPolarCommand,       0},
    {"param",
     "\r\nparam:\r\n Get a param data as a float to six decimal places [param vrefint_adc\r\n or param -list for all viewable param]\r\n",
     prvParamCommand,       1},
    {"sum",
     "\r\nsum:\r\n Calculate the sum of multiple numbers, such as sum 1, 2, 3, output: 6\r\n",
     prvsumCommand,         3},
    {"runtime",
    "\r\nruntime:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n",
     prvRunTimeStatsCommand, 0},
    {"ls",
    "\r\nls:\r\n List all included items in this file\r\n",
     prvListCommand, 0},
    {"cd",
    "\r\ncd:\r\n Change or enter the specified directory\r\n",
     prvCdCommand, 1},
    {"get",
    "\r\nget:\r\n Get a file variable value get xxx\r\n",
     prvGetVariableCommand, 1},
    {"set",
    "\r\nset:\r\n Set a file variable value set xxx yyy\r\n",
     prvSetVariableCommand, 2}
};


void CMD_RegisterCLICommands() {
    CurTreeNode = DataTreeRoot;
	for (int i = 0; i < CMD_COMMAND_NUM; i++) {
        FreeRTOS_CLIRegisterCommand(&xTaskStats[i]);	
    }
}


static BaseType_t prvTaskStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *const pcHeader = "     State   Priority  Stack    #\r\n************************************************\r\n";
    BaseType_t xSpacePadding;

	UNUSED(pcCommandString);
	UNUSED(xWriteBufferLen);
	configASSERT( pcWriteBuffer );

	strcpy(pcWriteBuffer, "Task");
	pcWriteBuffer += strlen(pcWriteBuffer);

	configASSERT(configMAX_TASK_NAME_LEN > 3);
	for(xSpacePadding = strlen("Task"); xSpacePadding < (configMAX_TASK_NAME_LEN - 3); xSpacePadding++) {
		*pcWriteBuffer = ' ';
		pcWriteBuffer++;
		*pcWriteBuffer = 0x00;
	}
	strcpy(pcWriteBuffer, pcHeader);
	vTaskList(pcWriteBuffer + strlen(pcHeader));

	return pdFALSE;
}


static BaseType_t prvPolarCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *const pcHeader = "\n#        Polaris License        #\r\n***************************************\r\n";
    UNUSED(pcCommandString);
	UNUSED(xWriteBufferLen);
	configASSERT( pcWriteBuffer );
    static uint8_t call_tick = 0;
    if (call_tick == 0) {
        strcpy(pcWriteBuffer, pcHeader);
        pcWriteBuffer += strlen(pcHeader);
        call_tick++;
        return pdTRUE;
    }
    else if (call_tick == 1) {
        char* struct_intr = "This is a motion robot that includes two types of motion structures (wheel legged and crawling legged)\r\n";
        strcpy(pcWriteBuffer, struct_intr);
        pcWriteBuffer += strlen(struct_intr);
        call_tick++;
        return pdTRUE;        
    }
    else if (call_tick >= 2) {
        char* intr = "Including various systems such as navigation, vision, path planning, and comprehensive control\r\n";
        strcpy(pcWriteBuffer, intr);
        pcWriteBuffer += strlen(intr);
        call_tick = 0;   
        return pdFALSE;     
    }
    else {
        call_tick = 0;
        return pdPASS;
    }
}


static BaseType_t prvParamCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *pcParameter;
    static UBaseType_t uxParameterNumber = 1;
    BaseType_t xParameterStringLength;

    configASSERT(pcWriteBuffer);
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, uxParameterNumber, &xParameterStringLength);
    configASSERT(pcParameter);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	sprintf(pcWriteBuffer, "%d: ", (int)uxParameterNumber);
	strncat(pcWriteBuffer, pcParameter, (size_t)xParameterStringLength);
	strncat(pcWriteBuffer, "\r\n", strlen("\r\n"));

    float vart = 2.5456465f;
    if (strcmp(pcParameter, "vart") == 0) {
        sprintf(pcWriteBuffer, ":  %f\r\n", vart);
        return pdFALSE;
    }
    else {
        sprintf(pcWriteBuffer, "\r\nThe variable was not found\r\n");
        return pdFALSE;
    }
}


static BaseType_t prvsumCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    static UBaseType_t uxParameterNumber = 1;
    static uint32_t sum_num = 0;

	configASSERT(pcWriteBuffer);

	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,	uxParameterNumber, &xParameterStringLength);

	if (pcParameter != NULL) {
		pcWriteBuffer[0] = 0x00;
		sum_num += atoi(pcParameter);
        uxParameterNumber++;
		return pdTRUE;
	}
	else {
		if(uxParameterNumber == 1) {
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strcpy(pcWriteBuffer, "no parameters!\r\n");			
		}
		else {
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			sprintf(pcWriteBuffer, "\r\nsum = %d \r\n", sum_num);	
			sum_num = 0;
		}
		uxParameterNumber = 1;
		return pdFALSE;
	}
}


static BaseType_t prvRunTimeStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char * const pcHeader = "  Abs Time      % Time\r\n****************************************\r\n";
    BaseType_t xSpacePadding;

	configASSERT(pcWriteBuffer);
	strcpy(pcWriteBuffer, "Task");
	pcWriteBuffer += strlen(pcWriteBuffer);
	for (xSpacePadding = strlen("Task"); xSpacePadding < (configMAX_TASK_NAME_LEN - 3); xSpacePadding++) {
		*pcWriteBuffer = ' ';
		pcWriteBuffer++;
		*pcWriteBuffer = 0x00;
	}
	strcpy(pcWriteBuffer, pcHeader);
	vTaskGetRunTimeStats(pcWriteBuffer + strlen(pcHeader));
	return pdFALSE;
}


static BaseType_t prvListCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	configASSERT(pcWriteBuffer);
    const char * tap = "\t\t";
    const char * new_line = "\r\n";
    static uint32_t line_tick = 0;

    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    strcpy(pcWriteBuffer, new_line);
    pcWriteBuffer += strlen(new_line);

	List_TypeDef* cur_list = Tree_GetNodeChildren(CurTreeNode);
    List_NodeTypeDef* cur_node = cur_list->head;
    for (int i = 0; i < cur_list->len; i++) {
        if (cur_node == NULL) return pdFALSE;
        Tree_NodeTypeDef* cur_tree = cur_node->val;

        if (cur_tree == NULL) return pdFALSE;
        if (Tree_IsDirNode(cur_tree)) {
            strcpy(pcWriteBuffer, "$");
            pcWriteBuffer += strlen("$");            
        }
        strcpy(pcWriteBuffer, cur_tree->name);
        pcWriteBuffer += strlen(cur_tree->name);
        strcpy(pcWriteBuffer, tap);
        pcWriteBuffer += strlen(tap);

        line_tick++;
        if (line_tick >= 3) {
            strcpy(pcWriteBuffer, new_line);
            pcWriteBuffer += strlen(new_line);
            line_tick = 0;
        }

        cur_node = cur_node->next;
    }

    if (line_tick != 0) {
        strcpy(pcWriteBuffer, new_line);
        pcWriteBuffer += strlen(new_line);
        line_tick = 0;
    }

    return pdFALSE;
}


static BaseType_t prvCdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *pcParameter;
    const char *no_path = "the path was not a dictionary:";
    const char *new_line = "\r\n";
    BaseType_t xParameterStringLength;
    UBaseType_t uxParameterNumber = 1;

	configASSERT(pcWriteBuffer);

	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,	uxParameterNumber, &xParameterStringLength);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    strcpy(pcWriteBuffer, new_line);			
    pcWriteBuffer += strlen(new_line); 
            
	if (pcParameter == NULL) {
		strcpy(pcWriteBuffer, no_path);
		return pdFALSE;
	}

	if(uxParameterNumber == 1) {
        Tree_NodeTypeDef* new_node = Tree_OpenNode(CurTreeNode, pcParameter);
        if ((new_node == NULL) || (Tree_IsDirNode(new_node))) {
		    strcpy(pcWriteBuffer, no_path);			
            pcWriteBuffer += strlen(no_path);      
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line);   
        }
        else {
            char *now_path_str = CMD_GetNowPathStr();
            CurTreeNode = new_node;
            memset(now_path_str, 0x00, NOW_PATH_LENGTH);
            Tree_GetNodePath(CurTreeNode, now_path_str);
        }
	}
    return pdFALSE;

}


static BaseType_t prvGetVariableCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *pcParameter;
    const char *no_path = "Can not get, the file was not a param:";
    const char *new_line = "\r\n";
    BaseType_t xParameterStringLength;
    UBaseType_t uxParameterNumber = 1;

	configASSERT(pcWriteBuffer);

	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,	uxParameterNumber, &xParameterStringLength);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    strcpy(pcWriteBuffer, new_line);			
    pcWriteBuffer += strlen(new_line); 
            
	if (pcParameter == NULL) {
		strcpy(pcWriteBuffer, no_path);
		return pdFALSE;
	}

	if(uxParameterNumber == 1) {
        Tree_NodeTypeDef* file_node = Tree_OpenNode(CurTreeNode, pcParameter);
        if ((file_node == NULL) || (!Tree_IsDirNode(file_node))) {
		    strcpy(pcWriteBuffer, no_path);			
            pcWriteBuffer += strlen(no_path);      
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line);   
        }
        else {
            char *ret_data = malloc(NOW_PATH_LENGTH);
            uint8_t ret = Tree_NodeDataToStr(ret_data, file_node);
		    if (ret == 0) {
                strcpy(pcWriteBuffer, pcParameter);			
                pcWriteBuffer += strlen(pcParameter);   
                strcpy(pcWriteBuffer, ": ");			
                pcWriteBuffer += strlen(": ");  
                strcpy(pcWriteBuffer, ret_data);			
                pcWriteBuffer += strlen(ret_data);  
                strcpy(pcWriteBuffer, new_line);			
                pcWriteBuffer += strlen(new_line);  
            }
            else if (ret >= 4) {
                char *failed_node = "Get the value failed: ";
		        strcpy(pcWriteBuffer, failed_node);			
                pcWriteBuffer += strlen(failed_node);  
                strcpy(pcWriteBuffer, pcParameter);			
                pcWriteBuffer += strlen(pcParameter);   
                strcpy(pcWriteBuffer, new_line);			
                pcWriteBuffer += strlen(new_line);    
            }
            else {
                char *no_node = "The file lost:";
		        strcpy(pcWriteBuffer, no_node);			
                pcWriteBuffer += strlen(no_node);  
                strcpy(pcWriteBuffer, pcParameter);			
                pcWriteBuffer += strlen(pcParameter);   
                strcpy(pcWriteBuffer, new_line);			
                pcWriteBuffer += strlen(new_line); 
            }

        }
	}
    return pdFALSE;
}


static BaseType_t prvSetVariableCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
    const char *pcParameter;
    const char *no_path = "Can not set, the file was not a param:";
    const char *new_line = "\r\n";
    BaseType_t xParameterStringLength;
    static UBaseType_t uxParameterNumber = 1;
    static Tree_NodeTypeDef* file_node;

	configASSERT(pcWriteBuffer);

	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString,	uxParameterNumber, &xParameterStringLength);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    strcpy(pcWriteBuffer, new_line);			
    pcWriteBuffer += strlen(new_line); 
            
	if (pcParameter == NULL) {
		strcpy(pcWriteBuffer, no_path);
		return pdFALSE;
	}

	if (uxParameterNumber == 1) {
        char *param_path = malloc(NOW_PATH_LENGTH);
        strncpy(param_path, pcParameter, xParameterStringLength);
        file_node = Tree_OpenNode(CurTreeNode, param_path);
        free(param_path);
        if ((file_node == NULL) || (!Tree_IsDirNode(file_node))) {
		    strcpy(pcWriteBuffer, no_path);			
            pcWriteBuffer += strlen(no_path);      
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line);   
            uxParameterNumber = 1;
            return pdFALSE;
        }
        else {
            uxParameterNumber = 2;
            return pdTRUE;
        }
	}
    if (uxParameterNumber == 2) {
        uint8_t ret = Tree_NodeDataFromStr(pcParameter, file_node);
		if (ret == 0) { 
            strcpy(pcWriteBuffer, file_node->name);			
            pcWriteBuffer += strlen(file_node->name);
            strcpy(pcWriteBuffer, " = ");			
            pcWriteBuffer += strlen(" = ");  
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line);  
        }
        else if (ret >= 4) {
            char *failed_node = "Set the value failed: ";
		    strcpy(pcWriteBuffer, failed_node);			
            pcWriteBuffer += strlen(failed_node);  
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line);    
        }
        else {
            char *no_node = "The file lost:";
		    strcpy(pcWriteBuffer, no_node);			
            pcWriteBuffer += strlen(no_node);  
            strcpy(pcWriteBuffer, pcParameter);			
            pcWriteBuffer += strlen(pcParameter);   
            strcpy(pcWriteBuffer, new_line);			
            pcWriteBuffer += strlen(new_line); 
        }
        uxParameterNumber = 1;
        return pdFALSE;
    }
	return pdFALSE;
}


char *CMD_GetNowPathStr() {
	return TREE_RET_PATH;
}
