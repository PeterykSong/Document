## 문제 1. 

$$ 
M = 
  \begin{bmatrix}
  cos\theta & -sin\theta cos \alpha & sin\theta sin \alpha & x \\ 
  sin\theta & cos\theta cos \alpha & -cos\theta sin \alpha & y \\ 
  0 & sin\alpha & cos\alpha  & z \\ 
  0 & 0 & 0 & 1 \\
  \end{bmatrix}

  = 
  \begin{bmatrix}
   -1 & 0 & 0 & 0 \\
    0 & 1 & 0 & 4\\
    0 & 0 & -1 & 1 \\
    0 & 0 & 0 & 1
  \end{bmatrix}
 
$$

$S_i$ 의 요소는 다음과 같다. 

| $i$ | $w_i$ | $v_i$ |
|:---:|:-------:|:-------:|
|  1 | (0,0,1)  | (0,0,0) |
|  2 | (1,0,0)  | (0,2,0) |
|  3 | (1,0,0)  | (0,2,-1)        |
|  4 | (0,0,0)  | (0,1,0)        |
|  5 | (0,$\frac{\sqrt{2}}{2}$,-$\frac{\sqrt{2}}{2}$) |($-\frac{5\sqrt{2}}{2}$,0,0)         |
|  6 | (0,1,0)  | (-2,0,0)   |

따라서, 
$$
S_1 = 
\begin{bmatrix} 
0\\
0\\
1\\
0\\
0\\
0\\
\end{bmatrix},

S_2 = 
\begin{bmatrix} 
1\\
0\\
0\\
0\\
2\\
0\\
\end{bmatrix},

S_3 = 
\begin{bmatrix} 
1\\
0\\
0\\
0\\
2\\
-1\\
\end{bmatrix},
S_4 = 
\begin{bmatrix} 
0\\
0\\
0\\
0\\
1\\
0\\
\end{bmatrix},
S_5 = 
\begin{bmatrix} 
0\\ \frac{\sqrt{2}}{2}\\-\frac{\sqrt{2}}{2}\\ -\frac{5\sqrt{2}}{2}\\0\\0\\   
\end{bmatrix},

S_6 = 
\begin{bmatrix} 
0\\
1\\
0\\
-2\\
0\\
0\\
\end{bmatrix}

$$

   
     
       
         


## 문제 4. 

### (a) $E(p,v)=\frac{1}{2}(v^2 + p^2)$ 임을 보이시오.  

$$ 
\frac{dE}{dt} = \frac{1}{2}\frac{d(v^2)}{dt} + \frac{1}{2}\frac{d(p^2)}{dt} = v\frac{dv}{dt} + p\frac{dp}{dt}=va+pv=v(-p)+pv =0
$$

따라서 시간에 따른 E의 변화가 없으므로, 보존된다. 

### (b) $\Phi_k$ 를 찾고, 에너지는 발산하는가

- Update Rule:
  
  $$
  p(k+1) = p(k) + v(k)h \\
  v(k+1) = v(k) + a(k) h
  $$
   
   단순조화진동하는 진자이므로, $a=-p$ 이므로, 
   $v(k+1)$을 다시 쓰면 다음과 같다. 
   $$
  p(k+1) = p(k) + v(k)h \\
  v(k+1) = v(k) - p(k) h
  $$

  이를 행렬로 표현하면, 

  $$
  \begin{bmatrix} 
  p(k+1) \\
  v(k+1)
  \end{bmatrix}
   = 
   \begin{bmatrix} 
  p(k) + v(k)h \\
   - p(k) h + v(k) 
   \end{bmatrix}
  = 
   \begin{bmatrix} 
   1 & h\\
   -h & 1
   \end{bmatrix}

   \begin{bmatrix} 
  p(k) \\
   v(k) 
   \end{bmatrix}
$$

   $\begin{bmatrix} 
  p(k) \\
   v(k) 
   \end{bmatrix} = X(k)
$ 라 할때 $k, k+1$일때의 에너지 식을 표현하면 아래와 같다. 

$$E(k) = \frac{1}{2}(p(k)^2 + v(k)^2) = \frac{1}{2}\|X(k)\|^2$$

$$E(k+1) = \frac{1}{2} \| \mathbf{x}_{k+1} \|^2 = \frac{1}{2} \| \Phi_k {X}(k) \|^2 =\frac{1}{2}X_k^T \Phi_k^T \Phi_k X_k$$

이때 $\Phi_k^T \Phi_k$를 계산해보면, 

$$
\Phi_k^T \Phi_k = 
\begin{bmatrix} 
   1 & h\\
   -h & 1
   \end{bmatrix}
\begin{bmatrix} 
   1 & -h\\
   h & 1
   \end{bmatrix}
=
\begin{bmatrix} 
   1+h^2 & 0\\
   0 & 1+h^2
   \end{bmatrix} = (1+h^2) I
$$
결국, 
$$
E(k+1) = (1+h^2)I  E(k) = (1+h^2)  E(k)
$$

결국 $1+h^2 > 1$ 이므로 발산한다. 


### (c) IEM 방식에서 $\Phi_k$ 를 찾고, 에너지는 감소하는가 증명하라

- Update Rule:
  
  $$
  p(k+1) = p(k) + v(k+1)h \\
  v(k+1) = v(k) + a(k+1) h
  $$
   
   단순조화진동하는 진자이므로, $a=-p$ 이므로, 
   $v(k+1)$을 다시 쓰면 다음과 같다. 
   $$
  p(k+1) = p(k) + v(k+1)h \\
  v(k+1) = v(k) - p(k+1) h
  $$

  위의 연립방정식을 풀면, 

  $$
    \begin{align}
    p(k+1) &= p(k) + v(k+1)h \\
           &= p(k) + (v(k) - p(k+1) h)h\\
           &= p(k) + v(k)h -p(k+1) h^2\\
    \end{align}
  $$

  $$
  (1+h^2) p(k+1)  = p(k) + v(k)h\\
  p(k+1) = \frac{p(k) + v(k)h}{(1+h^2)}\\
         = \frac{1}{1+h^2}p(k) + \frac{h}{1+h^2}v(k)\\
         =\frac{1}{1+h^2} (p(k)+v(k)h)
  $$

다음, $v(k+1)$에 대한 수식 중, $p(k+1)$ 에 해당하는 내용을 마찬가지로 치환하면

$$
v(k+1) = v(k) - (p(k) + v(k+1)h)h \\
       = \frac{1}{1+h^2} (- p(k)h+ v(k) )
$$

위 두항을 선형식으로 표현하면, 

$$
\begin{bmatrix} 
  p(k+1) \\
  v(k+1)
  \end{bmatrix} 
= \frac{1}{1+h^2} 

\begin{bmatrix} 
  1& h\\
  - h & 1 
  \end{bmatrix} 

  \begin{bmatrix} 
  p(k) \\
  v(k)
  \end{bmatrix}  \\
  $$

  (b) 문항에서의 계산과정을 참조하여,  $\Phi_k^T \Phi_k$를 계산해보면, 
$$
\Phi_k^T \Phi_k = (\frac{1}{1+h^2} )^2  
\begin{bmatrix} 
  1+h^2 & 0 \\
  0 & 1+h^2
  \end{bmatrix} 
  = (\frac{1}{1+h^2} ) I
$$
따라서 Step이 반복될수록 0에 수렴한다. 


### (d) SEM 에서 에너지 변화를 평가하라.

  $$
  p(k+1) = p(k) + v(k+1)h \\
  v(k+1) = v(k) + a(k) h
  $$

위 식을 다시 쓰면, 

$$
  p(k+1) = p(k) + v(k+1)h \\
  v(k+1) = v(k) -p(k) h
$$

이며, 각 항을 치환하며 선형식으로정리하면, 

$$
\begin{align}
p(k+1) &= p(k) + v(k+1)h \\
       &= p(k) + (v(k)-p(k)h)h \\
       &= (1-h^2)p(k) + v(k)h \\
\end{align}
$$
따라서 전체 식은 다음과 같이 표현된다. 


$$
\begin{bmatrix} 
  p(k+1) \\
  v(k+1)
  \end{bmatrix} 
= 

\begin{bmatrix} 
  1-h^2 & h\\
  - h & 1 
  \end{bmatrix} 

  \begin{bmatrix} 
  p(k) \\
  v(k)
  \end{bmatrix}  \\
  $$

에너지식을 이전 답에서 가져와 쓰면, 다음과 같이 계산되며, 
$$
E(k+1) = \frac{1}{2} \| \Phi_k \mathbf{x}_k \|^2 = \frac{1}{2} \mathbf{x}_k^\top \Phi_k^\top \Phi_k \mathbf{x}_k
$$


$$
\Phi_k^\top \Phi_k =
\begin{bmatrix}
1 - h^2 & -h \\
h & 1
\end{bmatrix}
\begin{bmatrix}
1 - h^2 & h \\
- h & 1
\end{bmatrix}
 =
\begin{bmatrix}
(1 - h^2)^2 + h^2 & (1 - h^2) h - h \\
(1 - h^2) h - h & h^2 + 1
\end{bmatrix} \\
=
\begin{bmatrix}
1 - h^2 + h^4 & - h^3 \\
- h^3 & 1 + h^2
\end{bmatrix}
$$

이때, 이 행렬은 파라미터의 값에 따라 Conditionally stable 하다고 할수 있다. 