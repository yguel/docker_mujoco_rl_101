# MuJoCo Docker Examples

Ce dossier contient des exemples pour tester l'installation MuJoCo dans le conteneur Docker.

## Fichiers disponibles :

### `test_mujoco.py`
Script de test basique pour vérifier que MuJoCo fonctionne correctement :
- Vérifie la version installée
- Crée un modèle simple (boîte qui tombe)
- Execute quelques pas de simulation
- Affiche les résultats

## Utilisation :

```bash
# Dans le conteneur Docker
python3 /workspace/examples/test_mujoco.py
```

## Ajout d'exemples :

Vous pouvez ajouter vos propres scripts d'exemple dans ce dossier. Ils seront automatiquement copiés dans le conteneur lors du build.