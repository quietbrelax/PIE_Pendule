# Simulation et contrôle d'un double pendule monté sur un chariot

Ce dépôt contient l'essentiel des ressources (articles et thèses, codes) jugées pertinentes par l'équipe 24-25.
Il permet de simuler, stabiliser et explorer le contrôle d’un double pendule monté sur un chariot en translation linéaire. 

## Quelques vidéos pour visualiser les méthodes existantes :
   - Théorie du contrôle sur triple pendule 56 transitions entre équilibres : https://www.youtube.com/watch?v=I5GvwWKkBmg&list=PLOKE46QBHgpoqFyFBukyGVAGmkRzaNfBK
   - Théorie du contrôle sur triple pendule, seulement l'équilibre vertical mais stabilité ahurissante : https://www.youtube.com/watch?v=cyN-CRNrb3E&list=PLOKE46QBHgpoqFyFBukyGVAGmkRzaNfBK&index=3
   - Exemple de la vitesse d'apprentissage de PILCO : https://www.youtube.com/watch?v=N-yrQu9zuOI&list=PLOKE46QBHgpoqFyFBukyGVAGmkRzaNfBK&index=4
   - Pour comprendre pourquoi les IA classiques mettent beaucoup trop longtemps à apprendre à stabiliser un double pendule : https://www.youtube.com/watch?v=9gQQAO4I1Ck&list=PLOKE46QBHgpoqFyFBukyGVAGmkRzaNfBK&index=3


## Structure du dépôt

Le dépôt est organisé en plusieurs dossiers :

### 1. **`simulation`**
   - **`Sim IHM V2.1.py`**  
     La simulation principale utilisant une intégration de **Runge-Kutta d'ordre 4**.  
     Cette simulation génère des résultats fluides du comportement d’un double pendule monté sur un chariot.
   - **`methode_lqr.py`**  
     Une implémentation de la méthode **LQR (Linear Quadratic Regulator)**.  
     Ce script stabilise le double pendule en position verticale en linéarisant les équations autour de la position d’équilibre.

### 2. **`pilco environment example`**
   - Ce dossier contient un exemple d’environnement pour appliquer l'algorithme **PILCO** (Probabilistic Inference for Learning Control).  
     L’objectif est d’adapter cet algorithme au contrôle du double pendule, mais le projet est encore en cours de développement et **n'est pas encore fonctionnel**.

   Principaux fichiers :
   - `environment.py` : Définit l'environnement personnalisé pour PILCO.
   - `pilco.py` : Contient l'implémentation principale de l'algorithme PILCO.
   - `policy.py` et `utils.py` : Modules d'outils complémentaires.

### 3. **`documentation`**
   - Ce dossier contient des **documents explicatifs** sur :
     - La **théorie du contrôle** pour le double pendule.
     - Une introduction et un guide sur **PILCO**.
     - Une autre approche d'apprentissage par renforcement

---

