#!/usr/bin/env python3
"""Atalho para planning/yaml_para_pddl.py (o codigo do conversor fica no pacote,
para o launch tambem conseguir usar). Uso igual ao de antes:
    python3 tools/yaml_para_pddl.py ATT2.yaml --start 2 --finish 1 --locations config/locations.yaml
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from planning.yaml_para_pddl import main  # noqa: E402

main()
