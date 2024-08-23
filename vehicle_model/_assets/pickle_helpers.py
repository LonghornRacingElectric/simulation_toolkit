from typing import Any
import pickle

def pickle_import(file_path: str) -> Any:
    with open(file_path, 'rb') as inp:
        imported_obj = pickle.load(inp)
    
    return imported_obj