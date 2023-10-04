# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Semantics, Usd
from typing import List, Tuple, Dict


def add_update_semantics(prim: Usd.Prim, semantic_label: str, type_label: str = "class", suffix="") -> None:
    """Apply a semantic label to a prim or update an existing label

    Args:
        prim (Usd.Prim): Usd Prim to add or update semantics on
        semantic_label (str): The label we want to apply
        type_label (str): The type of semantic information we are specifying (default = "class")
        suffix (str): Additional suffix used to specify multiple semantic attribute names.
        By default the semantic attribute name is "Semantics", and to specify additional
        attributes a suffix can be provided. Simple string concatenation is used :"Semantics" + suffix (default = "")
    """
    # Apply or acquire the existing SemanticAPI
    semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics" + suffix)
    if not semantic_api:
        semantic_api = Semantics.SemanticsAPI.Apply(prim, "Semantics" + suffix)
        semantic_api.CreateSemanticTypeAttr()
        semantic_api.CreateSemanticDataAttr()

    type_attr = semantic_api.GetSemanticTypeAttr()
    data_attr = semantic_api.GetSemanticDataAttr()

    # Set the type and data for the SemanticAPI
    if type_label is not None:
        type_attr.Set(type_label)
    if semantic_label is not None:
        data_attr.Set(semantic_label)
    return


def remove_all_semantics(prim: Usd.Prim, recursive: bool = False) -> None:
    """Removes all semantic tags from a given prim and its children

    Args:
        prim (Usd.Prim): Prim to remove any applied semantic APIs on
        recursive (bool, optional): Also traverse children and remove semantics recursively. Defaults to False.
    """

    def remove_semantics(input_prim: Usd.Prim):
        for prop in input_prim.GetProperties():
            is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(prop.GetPath())
            if is_semantic:
                name = prop.SplitName()[1]
                sem = Semantics.SemanticsAPI.Get(input_prim, name)

                typeAttr = sem.GetSemanticTypeAttr()
                dataAttr = sem.GetSemanticDataAttr()
                input_prim.RemoveProperty(typeAttr.GetName())
                input_prim.RemoveProperty(dataAttr.GetName())
                input_prim.RemoveAPI(Semantics.SemanticsAPI, name)

    if recursive:
        for p in Usd.PrimRange(prim.GetPrim()):
            remove_semantics(p)
    else:
        remove_semantics(prim)


def get_semantics(prim: Usd.Prim) -> Dict[str, Tuple[str, str]]:
    """Returns semantics that are applied to a prim
    
    Args:
        prim (Usd.Prim): Prim to return semantics for

    Returns:
        Dict[str, Tuple[str,str]]: Dictionary containing the name of the applied semantic, and the type and data associated with that semantic. 
    """
    result = {}
    for prop in prim.GetProperties():
        is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(prop.GetPath())
        if is_semantic:
            name = prop.SplitName()[1]
            sem = Semantics.SemanticsAPI.Get(prim, name)

            typeAttr = sem.GetSemanticTypeAttr()
            dataAttr = sem.GetSemanticDataAttr()
            result[name] = (typeAttr.Get(), dataAttr.Get())
    return result
