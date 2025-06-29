#include "object_dictionary.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/**
  * @brief     Create object dictionary
  * @return    Object dictionary pointer
  * @note      If creation fails, return NULL. If creation succeeds, return object dictionary pointer.
  */
ObjectDictionary* OD_Create(void) 
{
    ObjectDictionary* od = (ObjectDictionary*)malloc(sizeof(ObjectDictionary));
    if (od == NULL) {
        return NULL;
    }

    od->head = NULL;
    od->entryCount = 0;
    return od;
}

/**
  * @brief     Destroy object dictionary
  * @param[in] od: Object dictionary pointer
  * @note      If destruction fails, return. If destruction succeeds, return.
  */
void OD_Destroy(ObjectDictionary* od) 
{
    if (od == NULL) {
        return;
    }

    // traverse the linked list and release all nodes
    OD_Entry* current = od->head;
    while (current != NULL) {
        OD_Entry* next = current->next;
        if (current->data != NULL) {
            free(current->data);
        }
        free(current);
        current = next;
    }

    free(od);
}

/**
  * @brief     Find entry
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @return    Entry pointer
  * @note      If finding fails, return NULL. If finding succeeds, return entry pointer.
  */
static OD_Entry* OD_FindEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex) 
{
    if(od == NULL || od->head == NULL)
        return NULL;
        
    OD_Entry* current = od->head;
    while (current != NULL) {
        if (current->index == index && current->subindex == subindex) {
            return current;
        }
		if(current->next != NULL)
			current = current->next;
		else
			return NULL;
    }
    return NULL;
}

/**
  * @brief     Get data type size
  * @param[in] dataType: Data type
  * @return    Data type size
  * @note      If getting fails, return 0. If getting succeeds, return data type size.
  */
static size_t OD_GetDataTypeSize(OD_DataType dataType) 
{
    switch (dataType) {
        case OD_TYPE_BOOLEAN:
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            return 1;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            return 2;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
        case OD_TYPE_FLOAT32:
            return 4;
        case OD_TYPE_STRING:
            return 256; // default string length
        default:
            return 0;
    }
}

/**
  * @brief     Add entry
  * @param[in] od: Object dictionary pointer
  * @param[in] entry: Entry pointer
  * @return    true: Add entry successfully, false: Add entry failed
  * @note      If adding fails, return false. If adding succeeds, return true.
  */
bool OD_AddEntry(ObjectDictionary* od, OD_Entry* entry) 
{
    if (od == NULL || entry == NULL || OD_FindEntry(od, entry->index, entry->subindex) != NULL) {
        return false;
    }

    // allocate data space
    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    entry->data = malloc(dataSize);
    if (entry->data == NULL) {
        return false;
    }
    
    // 初始化内存为0
    memset(entry->data, 0, dataSize);

    // 将新节点插入到链表头部
    entry->next = od->head;
    od->head = entry;
    od->entryCount++;

    return true;
}

/**
  * @brief     Add entry
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @param[in] dataType: Data type
  * @param[in] data: Data pointer
  * @param[in] accessRead: Access read
  * @param[in] accessWrite: Access write
  * @return    true: Add entry successfully, false: Add entry failed
  * @note      If adding fails, return false. If adding succeeds, return true.
  */
bool OD_User_AddEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex, OD_DataType dataType, void* data, bool accessRead, bool accessWrite) 
{
    if (od == NULL) {
        return false;
    }
    
    OD_Entry* entry = (OD_Entry*)malloc(sizeof(OD_Entry));
    if (entry == NULL) {
        return false;
    }
    
    entry->index = index;
    entry->subindex = subindex;
    entry->dataType = dataType;
    entry->data = NULL;  // initialize to NULL, let OD_AddEntry allocate memory
    entry->accessRead = accessRead;
    entry->accessWrite = accessWrite;
    entry->next = NULL;
    
    bool result = OD_AddEntry(od, entry);
    if (result && data != NULL) {
        // if the addition is successful and data is not NULL, copy the data
        size_t dataSize = OD_GetDataTypeSize(dataType);
        if (dataSize > 0) {
            memcpy(entry->data, data, dataSize);
        }
    }
    
    return result;
}

/**
  * @brief     Initialize object dictionary
  * @param[in] od: Object dictionary pointer
  * @return    true: Initialize successfully, false: Initialize failed
  * @note      If initialization fails, return false. If initialization succeeds, return true.
  */
bool OD_User_Init(ObjectDictionary* od) 
{
    if (od == NULL) {
        return false;
    }
    
    uint8_t error_code = 0;
    uint32_t value = 0;
    
    // use the address of &value, but let OD_User_AddEntry handle the data copy internally
    error_code |= OD_User_AddEntry(od, OD_INDEX_ENCODER_DATA_RESET, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_PROFILE_SPEED, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT, OD_TYPE_INT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_POSITION, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_CURRENT, OD_SUBINDEX_DEFAULT, OD_TYPE_INT16, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_VELOCITY, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_INVERT_DIR, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_POS, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_TORQUE, OD_SUBINDEX_DEFAULT, OD_TYPE_INT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_MAX_SPEED_RPM, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_SPEED_LOOP, OD_SUBINDEX_1, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_SPEED_LOOP, OD_SUBINDEX_2, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_POSITION_LOOP, OD_SUBINDEX_1, OD_TYPE_INT16, &value, true, true);
    
    return error_code == 0;
}

/**
  * @brief     Remove entry
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @return    true: Remove entry successfully, false: Remove entry failed
  * @note      If removing fails, return false. If removing succeeds, return true.
  */
bool OD_RemoveEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex) 
{
    if (od == NULL || od->head == NULL) {
        return false;
    }

    OD_Entry* current = od->head;
    OD_Entry* prev = NULL;

    // if it is the head node
    if (current->index == index && current->subindex == subindex) {
        od->head = current->next;
        if (current->data != NULL) {
            free(current->data);
        }
        free(current);
        od->entryCount--;
        return true;
    }

    // find the node to be deleted
    while (current != NULL) {
        if (current->index == index && current->subindex == subindex) {
            prev->next = current->next;
            if (current->data != NULL) {
                free(current->data);
            }
            free(current);
            od->entryCount--;
            return true;
        }
        prev = current;
        current = current->next;
    }

    return false;
}

/**
  * @brief     Get entry
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @return    Entry pointer
  * @note      If getting fails, return NULL. If getting succeeds, return entry pointer.
  */
OD_Entry* OD_GetEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex)
 {
    return OD_FindEntry(od, index, subindex);
}

/**
  * @brief     Set value
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @param[in] value: Value pointer
  * @return    true: Set value successfully, false: Set value failed
  * @note      If setting fails, return false. If setting succeeds, return true.
  */
bool OD_SetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, uint8_t* value) 
{
    OD_Entry* entry = OD_FindEntry(od, index, subindex);
    if (entry == NULL) 
    {
        if(value == NULL)
            return false;
        return false;
    }

    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    memcpy(entry->data, value, dataSize);
    return true;
}

/**
  * @brief     Get value
  * @param[in] od: Object dictionary pointer
  * @param[in] index: Index
  * @param[in] subindex: Subindex
  * @param[in] value: Value pointer
  * @return    true: Get value successfully, false: Get value failed
  * @note      If getting fails, return false. If getting succeeds, return true.
  */
bool OD_GetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, uint8_t* value) 
{
    OD_Entry* entry = OD_FindEntry(od, index, subindex);
    if (entry == NULL) 
    {
        if(value == NULL)
            return false;
        return false;
    }

    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    memcpy(value, entry->data, dataSize);
    return true;
}

/**
  * @brief     Print all entries
  * @param[in] od: Object dictionary pointer
  * @note      If printing fails, return. If printing succeeds, return.
  */
void OD_PrintAll(ObjectDictionary* od) 
{
    if (od == NULL) {
        // printf("对象字典为空\n");
        return;
    }

    // printf("对象字典内容（共%d个条目）：\n", od->entryCount);
    OD_Entry* current = od->head;
    while (current != NULL) {
        // printf("索引: 0x%04X, 子索引: %d, 类型: %d, 描述: %s\n", 
            //    current->index, current->subindex, current->dataType, 
            //    current->description ? current->description : "无描述");
        current = current->next;
    }
} 
